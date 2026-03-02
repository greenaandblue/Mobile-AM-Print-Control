#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
motion_buffer_estimator.py

估算打印机 firmware buffer 中的剩余执行时间，用于预测暂停延迟。

核心原理：
    1. 解析即将发送的 G-code（G0/G1 运动指令）
    2. 基于运动学估算每条指令的执行时间
    3. 维护一个软件侧的 "影子队列"，模拟 firmware buffer
    4. 当调用 pause 时，返回预估的延迟时间

运动学模型：
    - 简化版：T = distance / feedrate
    - 进阶版：梯形速度曲线（考虑加速度）

用法：
    estimator = MotionBufferEstimator(
        buffer_size=16,          # firmware buffer 深度（经验值：Marlin 默认 16）
        max_accel=1000,          # mm/s² 最大加速度
        use_trapezoidal=True     # 是否使用梯形曲线
    )
    
    # 每次发送 G-code 前调用
    estimator.on_gcode_sent("G1 X100 Y50 F3000")
    
    # 每收到 'ok' 时调用
    estimator.on_gcode_ack()
    
    # 估算当前 buffer 剩余时间
    delay = estimator.estimate_pause_delay()
    print(f"预计暂停延迟: {delay:.3f} 秒")
"""

import re
import math
import time
from collections import deque
from typing import Optional, Dict, Tuple
from dataclasses import dataclass


@dataclass
class Move:
    """单条运动指令"""
    distance: float      # mm
    feedrate: float      # mm/s
    duration: float      # s（估算）
    timestamp: float     # 发送时刻
    gcode: str           # 原始指令（用于调试）


class MotionBufferEstimator:
    """
    firmware buffer 运动时间估算器。
    
    假设：
      - firmware 按 FIFO 顺序执行 G-code
      - buffer 满时会阻塞发送（printcore 已处理）
      - 每收到一个 'ok'，buffer 空出一个位置
    """
    
    def __init__(self,
                 buffer_size: int = 16,
                 max_accel: float = 1000.0,  # mm/s²
                 max_jerk: float = 10.0,      # mm/s（简化，用于估算启停损耗）
                 use_trapezoidal: bool = False,
                 debug: bool = False,
                 safety_margin: float = 1.2):  # 安全余量（20%），考虑firmware处理延迟等
        
        self.buffer_size     = buffer_size
        self.max_accel       = max_accel
        self.max_jerk        = max_jerk
        self.use_trapezoidal = use_trapezoidal
        self.debug           = debug
        self.safety_margin   = safety_margin  # 安全余量，用于补偿估算误差
        
        # 当前位置（mm）
        self._pos = {'X': 0.0, 'Y': 0.0, 'Z': 0.0, 'E': 0.0}
        
        # 当前 feedrate（mm/s，初始化为合理默认值）
        # 注意：实际打印时，第一个G-code通常会设置F参数
        # 如果没有F参数，feedrate会保持这个初始值
        self._feedrate = 50.0  # mm/s (3000 mm/min) - 默认值，应该很快被G-code中的F参数覆盖
        
        # 当前速度（mm/s）- 前一个move结束时的速度
        # 对于连续的move，速度是连续的，不会降到0
        self._current_velocity = 0.0  # 初始速度为0（打印开始）
        
        # 绝对 / 相对定位模式
        self._absolute_mode = True
        
        # 软件侧 buffer（影子队列）
        self._buffer = deque(maxlen=buffer_size)  # type: deque[Move]
        
        # 统计
        self.total_moves_sent = 0
        self.total_moves_acked = 0
    
    # ─────────────────────────────────────────────────────────────────────────
    # 核心接口
    # ─────────────────────────────────────────────────────────────────────────
    def on_gcode_sent(self, gcode: str):
        """
        当一条 G-code 被发送到打印机时调用。
        
        Args:
            gcode: 原始 G-code 字符串，如 "G1 X100 Y50 F3000"
        """
        stripped = gcode.strip()
        if not stripped:
            return
        
        # 解析指令
        parsed = self._parse_gcode(stripped)
        if not parsed:
            if self.debug:
                print(f"[BUFFER] SKIP (not parsed): {stripped[:50]}")
            return
        
        cmd = parsed.get('cmd', '')
        
        # G90 / G91（定位模式）
        if cmd == 'G90':
            self._absolute_mode = True
            return
        elif cmd == 'G91':
            self._absolute_mode = False
            return
        
        # G0 / G1（运动指令）
        if cmd in ('G0', 'G1'):
            move = self._compute_move(parsed, stripped)
            if move:
                if move.distance > 0.001:  # 忽略极小移动
                    self._buffer.append(move)
                    self.total_moves_sent += 1
                    
                    if self.debug:
                        print(f"[BUFFER] +MOVE: {stripped[:50]:50s} | "
                              f"dist={move.distance:7.2f}mm | "
                              f"feed={move.feedrate:7.1f}mm/s | "
                              f"time={move.duration:6.3f}s | "
                              f"buffer={len(self._buffer)}/{self.buffer_size}")
                else:
                    if self.debug:
                        print(f"[BUFFER] SKIP (distance too small): {stripped[:50]} | "
                              f"dist={move.distance:.6f}mm")
            else:
                if self.debug:
                    print(f"[BUFFER] SKIP (move=None): {stripped[:50]} | "
                          f"feedrate={self._feedrate:.2f}mm/s")
        elif self.debug:
            print(f"[BUFFER] SKIP (not G0/G1): {stripped[:50]} | cmd={cmd}")
    
    def on_gcode_ack(self):
        """
        当收到打印机的 'ok' 响应时调用（表示一条指令执行完毕）。
        
        注意：不是所有 'ok' 都对应一个 move，有些 G-code（如 G90/G91）不产生 move。
        但为了简化，我们假设每个 'ok' 对应一个 move。
        """
        if self._buffer:
            completed = self._buffer.popleft()
            self.total_moves_acked += 1
            
            # 更新当前速度（这个move已经执行完毕，速度应该是move结束时的速度）
            # 注意：这个速度已经在 _compute_move 中计算过了
            # 但我们需要从buffer中获取，因为buffer中的move已经包含了结束速度信息
            # 实际上，_current_velocity 已经在 _compute_move 中更新了，所以这里不需要再更新
            
            if self.debug:
                age = time.time() - completed.timestamp
                print(f"[BUFFER] -ACK : {completed.gcode[:50]:50s} | "
                      f"age={age:6.3f}s | "
                      f"buffer={len(self._buffer)}/{self.buffer_size} | "
                      f"current_vel={self._current_velocity:.2f}mm/s")
        elif self.debug:
            # Buffer 为空但收到了 'ok'，可能是非 move 指令的响应
            print(f"[BUFFER] -ACK : (buffer empty, non-move command?) | "
                  f"pending={self.total_moves_sent - self.total_moves_acked}")
    
    def estimate_pause_delay(self) -> float:
        """
        估算当前 buffer 中剩余的执行时间（即暂停延迟）。
        
        注意：实际暂停延迟可能比估算值长，因为：
          - firmware 处理延迟
          - 通信延迟
          - 加速度可能比设置值低
          - 需要考虑 cornering、jerk 等因素
        
        Returns:
            float: 预计延迟时间（秒），已应用安全余量
        """
        total_time = sum(move.duration for move in self._buffer)
        
        # 应用安全余量（考虑firmware处理延迟、通信延迟等）
        # 对于很短的延迟，添加一个最小延迟（firmware处理时间）
        if total_time > 0:
            # 应用安全余量
            estimated_time = total_time * self.safety_margin
            # 如果估算时间很短，添加一个最小延迟（firmware处理时间，通常50-200ms）
            if estimated_time < 0.5:
                estimated_time = max(estimated_time, 0.2)  # 至少200ms
        else:
            estimated_time = total_time
        
        if self.debug:
            if total_time > 0:
                print(f"[ESTIMATE] Pause delay: {estimated_time:.3f}s "
                      f"(raw={total_time:.3f}s, margin={self.safety_margin:.1f}x) "
                      f"({len(self._buffer)} moves in buffer)")
            else:
                print(f"[ESTIMATE] Pause delay: 0.000s "
                      f"({len(self._buffer)} moves in buffer) | "
                      f"feedrate={self._feedrate:.2f}mm/s | "
                      f"pending={self.total_moves_sent - self.total_moves_acked}")
        
        return estimated_time
    
    def get_buffer_status(self) -> Dict:
        """返回当前 buffer 状态（用于监控）"""
        return {
            'buffer_count':    len(self._buffer),
            'buffer_capacity': self.buffer_size,
            'buffer_usage':    f"{len(self._buffer) / self.buffer_size * 100:.1f}%",
            'pause_delay_s':   self.estimate_pause_delay(),
            'total_sent':      self.total_moves_sent,
            'total_acked':     self.total_moves_acked,
            'pending':         self.total_moves_sent - self.total_moves_acked,
            'current_feedrate': self._feedrate,
            'current_pos':     self._pos.copy(),
        }
    
    def get_detailed_diagnosis(self) -> Dict:
        """返回详细的诊断信息（用于调试）"""
        buffer_moves = []
        for i, move in enumerate(self._buffer):
            buffer_moves.append({
                'index': i,
                'gcode': move.gcode[:50],
                'distance': move.distance,
                'feedrate': move.feedrate,
                'duration': move.duration,
                'age': time.time() - move.timestamp,
            })
        
        return {
            'buffer_size': len(self._buffer),
            'buffer_capacity': self.buffer_size,
            'total_delay': self.estimate_pause_delay(),
            'current_feedrate_mm_s': self._feedrate,
            'current_position': self._pos.copy(),
            'total_moves_sent': self.total_moves_sent,
            'total_moves_acked': self.total_moves_acked,
            'pending_moves': self.total_moves_sent - self.total_moves_acked,
            'buffer_moves': buffer_moves,
        }
    
    # ─────────────────────────────────────────────────────────────────────────
    # G-code 解析
    # ─────────────────────────────────────────────────────────────────────────
    def _parse_gcode(self, line: str) -> Optional[Dict]:
        """
        解析 G-code 行，提取：
          - 指令类型（G0, G1, G90, G91, ...）
          - 坐标参数（X, Y, Z, E）
          - 速度参数（F）
        
        返回 dict 或 None（如果是注释/空行）
        """
        # 去除注释
        if ';' in line:
            line = line.split(';')[0]
        line = line.strip()
        
        if not line:
            return None
        
        # 提取 G 指令（支持行首有空格的情况）
        # 使用 search 而不是 match，以支持行首有空格的情况
        cmd_match = re.search(r'(G\d+|M\d+)', line, re.IGNORECASE)
        if not cmd_match:
            return None
        
        cmd = cmd_match.group(1).upper()
        
        # 提取参数（X, Y, Z, E, F）
        params = {}
        for axis in ['X', 'Y', 'Z', 'E']:
            match = re.search(rf'{axis}([-+]?\d*\.?\d+)', line, re.IGNORECASE)
            if match:
                try:
                    params[axis] = float(match.group(1))
                except ValueError:
                    if self.debug:
                        print(f"[WARNING] Failed to parse {axis} parameter: {match.group(1)}")
        
        # 提取 F（feedrate，单位 mm/min）
        f_match = re.search(r'F([-+]?\d*\.?\d+)', line, re.IGNORECASE)
        if f_match:
            try:
                params['F'] = float(f_match.group(1))
            except ValueError:
                if self.debug:
                    print(f"[WARNING] Failed to parse F parameter: {f_match.group(1)}")
        
        return {'cmd': cmd, 'params': params}
    
    # ─────────────────────────────────────────────────────────────────────────
    # 运动学计算
    # ─────────────────────────────────────────────────────────────────────────
    def _compute_move(self, parsed: Dict, gcode: str) -> Optional[Move]:
        """
        根据解析后的 G-code 计算运动参数。
        
        返回：
          - Move 对象（包含 distance, feedrate, duration）
          - None（如果不是有效运动）
        """
        params = parsed['params']
        
        # 更新 feedrate（如果有 F 参数）
        if 'F' in params:
            self._feedrate = params['F'] / 60.0  # mm/min → mm/s
        
        # 计算目标位置
        target = self._pos.copy()
        
        for axis in ['X', 'Y', 'Z', 'E']:
            if axis in params:
                if self._absolute_mode:
                    target[axis] = params[axis]
                else:
                    target[axis] += params[axis]
        
        # 计算位移
        dx = target['X'] - self._pos['X']
        dy = target['Y'] - self._pos['Y']
        dz = target['Z'] - self._pos['Z']
        de = target['E'] - self._pos['E']
        
        distance = math.sqrt(dx**2 + dy**2 + dz**2)
        
        # 如果只有挤出（retract/extrude），距离为 0，使用 E 轴位移
        if distance < 0.001 and abs(de) > 0.001:
            distance = abs(de)
        
        # 更新位置
        self._pos = target
        
        # 估算时间
        if self._feedrate <= 0:
            if self.debug:
                print(f"[WARNING] Feedrate <= 0: {self._feedrate:.2f} mm/s, skipping move")
            return None
        
        if distance <= 0:
            # 距离为0，可能是位置没有变化的G-code（如只设置F参数）
            if self.debug:
                print(f"[INFO] Distance = 0, skipping move (may be feedrate-only command)")
            return None
        
        if self.use_trapezoidal:
            duration = self._estimate_trapezoidal_time(distance, self._feedrate, self._current_velocity)
            # 更新当前速度（这个move结束时的速度）
            # 对于连续的move，结束速度通常是目标feedrate（除非距离太短）
            self._current_velocity = min(self._feedrate, self._estimate_end_velocity(distance, self._feedrate, self._current_velocity))
        else:
            duration = distance / self._feedrate  # 简化：匀速
            self._current_velocity = self._feedrate  # 匀速运动，结束速度等于feedrate
        
        move = Move(
            distance  = distance,
            feedrate  = self._feedrate,
            duration  = duration,
            timestamp = time.time(),
            gcode     = gcode
        )
        
        return move
    
    def _estimate_trapezoidal_time(self, distance: float, target_feedrate: float, initial_velocity: float) -> float:
        """
        梯形速度曲线估算（考虑加速度和速度连续性）。
        
        改进：考虑速度连续性，连续的move之间速度不会降到0。
        
        假设：
          - 初速度 v0 = initial_velocity（前一个move结束时的速度）
          - 目标速度 v_target = target_feedrate
          - 加速度 a = max_accel
        
        情况1：需要加速（v0 < v_target）
          - 加速到 v_target 需要时间 t_accel = (v_target - v0) / a
          - 加速段距离 d_accel = (v_target^2 - v0^2) / (2 * a)
          - 如果距离足够，加速后匀速；如果距离不足，只能加速到某个速度
        
        情况2：需要减速（v0 > v_target）
          - 类似，但方向相反
        
        情况3：速度不变（v0 ≈ v_target）
          - 匀速运动
        """
        v0 = initial_velocity
        v_target = target_feedrate
        a = self.max_accel
        
        # 如果速度已经接近目标速度（误差 < 1%），视为匀速
        if abs(v0 - v_target) < 0.01 * v_target:
            return distance / v_target if v_target > 0 else float('inf')
        
        # 情况1：需要加速
        if v0 < v_target:
            # 加速到目标速度需要的距离
            d_accel_needed = (v_target**2 - v0**2) / (2 * a)
            
            if distance <= d_accel_needed:
                # 距离不足，只能加速到某个速度 v_final
                # v_final^2 = v0^2 + 2 * a * distance
                v_final = math.sqrt(v0**2 + 2 * a * distance)
                t_total = (v_final - v0) / a
            else:
                # 距离足够，先加速到目标速度，然后匀速
                t_accel = (v_target - v0) / a
                d_const = distance - d_accel_needed
                t_const = d_const / v_target
                t_total = t_accel + t_const
        
        # 情况2：需要减速
        else:  # v0 > v_target
            # 减速到目标速度需要的距离
            d_decel_needed = (v0**2 - v_target**2) / (2 * a)
            
            if distance <= d_decel_needed:
                # 距离不足，只能减速到某个速度 v_final
                # v_final^2 = v0^2 - 2 * a * distance
                v_final = math.sqrt(max(0, v0**2 - 2 * a * distance))
                t_total = (v0 - v_final) / a if v_final > 0 else v0 / a
            else:
                # 距离足够，先减速到目标速度，然后匀速
                t_decel = (v0 - v_target) / a
                d_const = distance - d_decel_needed
                t_const = d_const / v_target
                t_total = t_decel + t_const
        
        return t_total
    
    def _estimate_end_velocity(self, distance: float, target_feedrate: float, initial_velocity: float) -> float:
        """
        估算move结束时的速度（用于下一个move的速度连续性）。
        """
        v0 = initial_velocity
        v_target = target_feedrate
        a = self.max_accel
        
        if abs(v0 - v_target) < 0.01 * v_target:
            return v_target
        
        if v0 < v_target:
            # 加速情况
            d_accel_needed = (v_target**2 - v0**2) / (2 * a)
            if distance <= d_accel_needed:
                # 距离不足，只能加速到某个速度
                return math.sqrt(v0**2 + 2 * a * distance)
            else:
                # 距离足够，加速到目标速度
                return v_target
        else:
            # 减速情况
            d_decel_needed = (v0**2 - v_target**2) / (2 * a)
            if distance <= d_decel_needed:
                # 距离不足，只能减速到某个速度
                return math.sqrt(max(0, v0**2 - 2 * a * distance))
            else:
                # 距离足够，减速到目标速度
                return v_target
    
    # ─────────────────────────────────────────────────────────────────────────
    # 重置
    # ─────────────────────────────────────────────────────────────────────────
    def reset(self):
        """重置所有状态（例如打印完成后）"""
        self._pos = {'X': 0.0, 'Y': 0.0, 'Z': 0.0, 'E': 0.0}
        self._feedrate = 50.0
        self._current_velocity = 0.0  # 重置速度
        self._absolute_mode = True
        self._buffer.clear()
        self.total_moves_sent = 0
        self.total_moves_acked = 0


# ─────────────────────────────────────────────────────────────────────────────
# 使用示例
# ─────────────────────────────────────────────────────────────────────────────
if __name__ == '__main__':
    """测试脚本：模拟发送一些 G-code，查看 buffer 状态"""
    
    print("=" * 70)
    print("Motion Buffer Estimator - 测试")
    print("=" * 70)
    
    # 创建估算器
    estimator = MotionBufferEstimator(
        buffer_size=8,            # 假设 buffer 深度为 8
        max_accel=1000.0,         # mm/s²
        use_trapezoidal=True,     # 使用梯形曲线
        debug=True                # 打印详细日志
    )
    
    # 模拟发送一些 G-code
    test_gcode = [
        "G90",                    # 绝对定位
        "G28",                    # 归零（忽略）
        "G1 X10 Y10 F3000",       # 移动到 (10, 10)，速度 3000 mm/min
        "G1 X20 Y20",             # 移动到 (20, 20)
        "G1 X30 Y30",
        "G1 X40 Y40",
        "G1 X50 Y50",
        "G1 X60 Y60",
        "G1 X70 Y70",
        "G1 X80 Y80",
    ]
    
    print("\n[发送 G-code]")
    for gcode in test_gcode:
        estimator.on_gcode_sent(gcode)
    
    print(f"\n[当前 buffer 状态]")
    status = estimator.get_buffer_status()
    for k, v in status.items():
        print(f"  {k:20s}: {v}")
    
    print(f"\n[预计暂停延迟]")
    delay = estimator.estimate_pause_delay()
    print(f"  如果现在调用 pause()，预计延迟: {delay:.3f} 秒")
    
    print(f"\n[模拟接收 'ok' 响应]")
    for i in range(3):
        estimator.on_gcode_ack()
        delay = estimator.estimate_pause_delay()
        print(f"  ACK #{i+1} | 剩余延迟: {delay:.3f} 秒 | "
              f"buffer: {len(estimator._buffer)}/{estimator.buffer_size}")
    
    print("\n" + "=" * 70)
    print("测试完成")
    print("=" * 70)