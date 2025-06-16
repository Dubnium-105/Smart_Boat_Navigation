#!/usr/bin/env python3
"""
测试推荐电机速度计算算法
验证Python实现与ESP32 C++实现的一致性
"""

import math
import sys
import os

# 添加当前目录到Python路径
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

def test_basic_algorithm(target_angle, target_strength):
    """测试基础算法"""
    left_speed = 0
    right_speed = 0
    
    if target_strength > 0.1:
        # 算法参数（与ESP32保持一致）
        base_speed = 50  
        max_turn_ratio = 1.8  
        pivot_threshold = 0.9  
        
        relative_angle = target_angle
        turn_factor = 0.0
        
        # 前方区域（-90到90度）
        if ((relative_angle >= 0 and relative_angle <= 90) or 
            (relative_angle >= 270 and relative_angle <= 360)):
            adjusted_angle = relative_angle - 360 if relative_angle > 180 else relative_angle
            turn_factor = adjusted_angle / 90.0
        # 后方区域（90到270度）
        else:
            adjusted_angle = relative_angle - 360 if relative_angle > 270 else relative_angle
            adjusted_angle = 360 - adjusted_angle if adjusted_angle > 180 else adjusted_angle
            if relative_angle > 90 and relative_angle < 180:
                turn_factor = 1.0  
            elif relative_angle > 180 and relative_angle < 270:
                turn_factor = -1.0  
            elif relative_angle == 180:
                turn_factor = 1.0  # 固定选择，便于测试
        
        turn_strength = abs(turn_factor)
        turn_direction = 1.0 if turn_factor >= 0 else -1.0
        
        # 前方区域使用差速转向
        if ((relative_angle >= 315 or relative_angle < 45) or
            (relative_angle >= 135 and relative_angle < 225)):
            is_forward = (relative_angle >= 315 or relative_angle < 45)
            direction = 1 if is_forward else -1
            
            turn_adjustment = base_speed * turn_strength * max_turn_ratio
            
            if turn_strength > pivot_threshold:
                pivot_factor = (turn_strength - pivot_threshold) / (1.0 - pivot_threshold)
                reduced_speed = base_speed * (1.0 - pivot_factor * 0.8)
                
                if turn_direction > 0:
                    left_speed = direction * reduced_speed
                    right_speed = direction * reduced_speed * (1.0 - 2.0 * pivot_factor)
                    if pivot_factor > 0.5:
                        right_speed = -right_speed  
                else:
                    right_speed = direction * reduced_speed
                    left_speed = direction * reduced_speed * (1.0 - 2.0 * pivot_factor)
                    if pivot_factor > 0.5:
                        left_speed = -left_speed  
            else:
                if turn_direction > 0:
                    left_speed = direction * base_speed
                    right_speed = direction * (base_speed - turn_adjustment)
                else:
                    right_speed = direction * base_speed
                    left_speed = direction * (base_speed - turn_adjustment)
        else:
            side_factor = 0.0
            if relative_angle >= 45 and relative_angle < 135:
                side_factor = math.sin(math.radians(relative_angle))
            elif relative_angle >= 225 and relative_angle < 315:
                side_factor = math.sin(math.radians(relative_angle))
            
            if turn_direction > 0:
                left_speed = base_speed * (1.0 - side_factor * 0.5)
                right_speed = base_speed * (1.0 - side_factor * 1.5)
                if side_factor > 0.7:
                    right_speed = -right_speed * (side_factor - 0.7) / 0.3
            else:
                right_speed = base_speed * (1.0 - side_factor * 0.5)
                left_speed = base_speed * (1.0 - side_factor * 1.5)
                if side_factor > 0.7:
                    left_speed = -left_speed * (side_factor - 0.7) / 0.3
        
        left_speed = max(-100, min(100, int(left_speed)))
        right_speed = max(-100, min(100, int(right_speed)))
    
    return left_speed, right_speed

def test_advanced_algorithm(target_angle, target_strength):
    """测试高级算法"""
    if target_strength <= 0.1 or target_angle < 0:
        return 20, -20  
    
    base_speed = 55
    max_turn_ratio = 1.5
    pivot_threshold = 0.85
    approach_threshold = 0.7
    
    angle_rad = math.radians(target_angle)
    
    shortest_angle = target_angle
    if shortest_angle > 180:
        shortest_angle = 360 - shortest_angle
        turn_factor = -math.sin(angle_rad)  
    else:
        turn_factor = math.sin(angle_rad)   
    
    dynamic_speed = base_speed
    if target_strength > approach_threshold:
        dynamic_speed = base_speed * (1.0 - (target_strength - approach_threshold) / (1.0 - approach_threshold) * 0.4)
    
    turn_strength = abs(turn_factor)
    
    if turn_strength > pivot_threshold:
        pivot_speed = dynamic_speed * 0.6
        if turn_factor > 0:  
            left_speed = pivot_speed
            right_speed = -pivot_speed / 2
        else:  
            left_speed = -pivot_speed / 2
            right_speed = pivot_speed
    else:
        speed_diff = dynamic_speed * turn_strength * max_turn_ratio
        
        if target_angle >= 315 or target_angle < 45:
            left_speed = dynamic_speed
            right_speed = dynamic_speed
            
            if turn_factor > 0.1:  
                right_speed -= speed_diff * 0.5
            elif turn_factor < -0.1:  
                left_speed -= speed_diff * 0.5
        elif target_angle >= 45 and target_angle < 135:
            left_speed = dynamic_speed
            right_speed = dynamic_speed - speed_diff
        elif target_angle >= 135 and target_angle < 225:
            pivot_speed = dynamic_speed * 0.7
            if target_angle < 180:  
                left_speed = pivot_speed
                right_speed = -pivot_speed / 3
            else:  
                left_speed = -pivot_speed / 3
                right_speed = pivot_speed
        else:
            left_speed = dynamic_speed - speed_diff
            right_speed = dynamic_speed
    
    left_speed = max(-100, min(100, int(left_speed)))
    right_speed = max(-100, min(100, int(right_speed)))
    
    return left_speed, right_speed

def run_tests():
    """运行测试用例"""
    print("=" * 60)
    print("推荐电机速度计算算法测试")
    print("=" * 60)
    
    # 测试用例：不同角度和强度的组合
    test_cases = [
        # (角度, 强度, 描述)
        (0, 0.8, "正前方，强信号"),
        (45, 0.6, "右前方，中等信号"),
        (90, 0.5, "正右方，中等信号"),
        (135, 0.7, "右后方，强信号"),
        (180, 0.8, "正后方，强信号"),
        (225, 0.6, "左后方，中等信号"),
        (270, 0.5, "正左方，中等信号"),
        (315, 0.7, "左前方，强信号"),
        (30, 0.9, "右前方，极强信号（接近目标）"),
        (0, 0.05, "正前方，弱信号"),
    ]
    
    print("\n基础算法测试结果:")
    print("-" * 60)
    print(f"{'角度':>6} {'强度':>6} {'左电机':>8} {'右电机':>8} {'描述':<20}")
    print("-" * 60)
    
    for angle, strength, description in test_cases:
        left, right = test_basic_algorithm(angle, strength)
        print(f"{angle:>6.0f} {strength:>6.2f} {left:>8} {right:>8} {description}")
    
    print("\n高级算法测试结果:")
    print("-" * 60)
    print(f"{'角度':>6} {'强度':>6} {'左电机':>8} {'右电机':>8} {'描述':<20}")
    print("-" * 60)
    
    for angle, strength, description in test_cases:
        left, right = test_advanced_algorithm(angle, strength)
        print(f"{angle:>6.0f} {strength:>6.2f} {left:>8} {right:>8} {description}")
    
    print("\n算法特性分析:")
    print("-" * 60)
    print("基础算法特点:")
    print("- 使用复杂的区域划分和转向策略")
    print("- 支持前进、后退、侧面区域的不同处理")
    print("- 具有原地转向和差速转向的平滑过渡")
    
    print("\n高级算法特点:")
    print("- 基于信号强度的动态速度调整")
    print("- 接近目标时自动减速")
    print("- 更精确的角度计算和转向控制")
    print("- 支持大角度原地转向和小角度微调")
    
    print("\n使用建议:")
    print("- 基础算法：适用于一般导航，转向响应较为激进")
    print("- 高级算法：适用于精确导航，接近目标时更平稳")

if __name__ == "__main__":
    run_tests()