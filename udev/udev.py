#!/usr/bin/env python3

import subprocess
import os
import time
import shutil
import glob
import sys

def clear_screen():
    """清空终端屏幕"""
    os.system('cls' if os.name == 'nt' else 'clear')

def print_header():
    """打印程序标题"""
    clear_screen()
    print("=" * 60)
    print("              ROS2 设备 UDEV 规则管理器")
    print("=" * 60)
    print("此工具可帮助您创建和管理设备的 udev 规则，为各种传感器创建永久性设备链接")
    print("-" * 60)

def get_device_descriptor(device_path):
    """获取设备的内核描述符"""
    try:
        result = subprocess.run(
            ["udevadm", "info", "--attribute-walk", "--name", device_path],
            capture_output=True, text=True, check=True
        )
        kernels_lines = [line for line in result.stdout.split('\n') if 'KERNELS==' in line]
        if len(kernels_lines) > 1:
            return kernels_lines[1].split('"')[1]  # 获取第二个 KERNELS== 的值
        else:
            print(f"未找到足够的 KERNELS== 行: {kernels_lines}")
    except subprocess.CalledProcessError as e:
        print(f"获取设备描述符时出错: {e}")
    return None

def create_udev_rule(device_name, descriptor, output_dir):
    """创建udev规则文件"""
    rule = f'KERNELS=="{descriptor}", MODE:="0666", SYMLINK+="{device_name}"'
    rule_file = os.path.join(output_dir, f'{device_name}.rules')
    try:
        with open(rule_file, 'w') as f:
            f.write(rule + '\n')
        print(f"已创建 {device_name} 的udev规则: {rule_file}")
        return True
    except IOError as e:
        print(f"写入 {device_name} 的udev规则时出错: {e}")
    except Exception as e:
        print(f"写入 {device_name} 的udev规则时发生意外错误: {e}")
    return False

def read_udev_rules(directory):
    """读取目录中的udev规则"""
    rules = {}
    try:
        if os.path.exists(directory):
            for filename in os.listdir(directory):
                if filename.endswith('.rules'):
                    try:
                        with open(os.path.join(directory, filename), 'r') as f:
                            rules[filename] = f.read()
                    except Exception as e:
                        print(f"读取文件 {filename} 时出错: {e}")
    except Exception as e:
        print(f"读取目录 {directory} 时出错: {e}")
    return rules

def detect_usb_device(device_name, device_symlink):
    """交互式检测USB设备"""
    print(f"\n正在设置 {device_name} 的udev规则...")
    print(f"该设备将使用符号链接: /dev/{device_symlink}")
    
    # 提示用户操作
    input(f"请确保 {device_name} 已断开连接，然后按Enter继续...")
    
    # 获取初始设备列表
    initial_devices = set(os.listdir('/dev'))
    print(f"初始设备列表已记录，现在请插入 {device_name}...")
    
    # 轮询以检测新设备
    countdown = 30  # 30秒超时
    while countdown > 0:
        time.sleep(1)
        countdown -= 1
        sys.stdout.write(f"\r等待设备连接... {countdown}秒 (Ctrl+C取消)")
        sys.stdout.flush()
        
        current_devices = set(os.listdir('/dev'))
        new_devices = current_devices - initial_devices
        
        # 检查新的ttyUSB或ttyACM设备
        for new_dev in new_devices:
            if new_dev.startswith('ttyUSB') or new_dev.startswith('ttyACM'):
                device_path = f'/dev/{new_dev}'
                print(f"\n\n检测到新设备: {device_path}")
                
                # 确认是否为正确设备
                confirm = input(f"这是您的 {device_name} 设备吗? (y/n): ")
                if confirm.lower() == 'y':
                    descriptor = get_device_descriptor(device_path)
                    if descriptor:
                        return device_symlink, descriptor
                    else:
                        print(f"无法获取设备 {device_path} 的描述符")
                        return None, None
        
        # 检查是否手动取消
        if countdown % 5 == 0:  # 每5秒询问一次
            check = input("\n\n未检测到新设备，是否继续等待? (y/n): ")
            if check.lower() != 'y':
                print("已取消设备检测")
                return None, None
    
    print("\n\n等待超时。未检测到新设备。")
    return None, None

def install_device_rule():
    """安装设备udev规则的交互流程"""
    print_header()
    print("添加新设备规则")
    print("-" * 60)
    print("请输入设备信息:")
    print("(输入 0 可返回主菜单)")
    print()
    
    # 获取用户自定义设备名称
    device_name = input("请输入设备描述名称 (例如: 激光雷达, 编码器): ").strip()
    if not device_name or device_name == "0":
        return
    
    # 获取设备符号链接名
    device_symlink = input(f"请输入设备符号链接名 (将作为 /dev/XXX 的XXX部分，如laser, encoder): ").strip()
    if not device_symlink or device_symlink == "0":
        return
    
    # 检查符号链接名是否合法
    if not device_symlink.replace('_', '').isalnum():
        print("符号链接名无效! 只能包含字母、数字和下划线")
        input("按Enter继续...")
        return
    
    # 创建临时目录存放规则文件
    output_dir = os.path.join(os.getcwd(), "temp_rules")
    os.makedirs(output_dir, exist_ok=True)
    
    # 检测设备并创建规则
    symlink, descriptor = detect_usb_device(device_name, device_symlink)
    if not symlink or not descriptor:
        print("未能成功检测设备或获取描述符")
        input("按Enter继续...")
        return
    
    # 创建规则文件
    if create_udev_rule(symlink, descriptor, output_dir):
        print(f"\n已成功为 {device_name} 创建规则")
        
        # 询问用户是否安装规则
        install = input("\n是否立即安装此规则到系统? (y/n): ")
        if install.lower() == 'y':
            try:
                rule_file = os.path.join(output_dir, f"{symlink}.rules")
                dest_file = f"/etc/udev/rules.d/{symlink}.rules"
                
                # 检查是否需要sudo权限
                try:
                    shutil.copy(rule_file, dest_file)
                except PermissionError:
                    print("需要管理员权限安装规则...")
                    subprocess.run(["sudo", "cp", rule_file, dest_file], check=True)
                
                print("正在重新加载udev规则...")
                subprocess.run(["sudo", "udevadm", "control", "--reload-rules"], check=True)
                subprocess.run(["sudo", "udevadm", "trigger"], check=True)
                print(f"\n规则已成功安装! 设备将以 /dev/{symlink} 的形式出现")
            except Exception as e:
                print(f"安装规则时出错: {e}")
                print("\n请手动安装规则:")
                print(f"sudo cp {os.path.join(output_dir, symlink+'.rules')} /etc/udev/rules.d/")
                print("sudo udevadm control --reload-rules")
                print("sudo udevadm trigger")
        else:
            print(f"\n规则文件已创建但未安装: {os.path.join(output_dir, symlink+'.rules')}")
            print("您可以稍后手动安装此规则")
    
    input("\n按Enter返回主菜单...")

def list_installed_rules():
    """列出已安装的udev规则"""
    print_header()
    print("已安装的设备规则:\n")
    
    rules_dir = "/etc/udev/rules.d/"
    rules = read_udev_rules(rules_dir)
    
    if not rules:
        print("未找到任何已安装的规则文件")
    else:
        for i, (filename, content) in enumerate(rules.items(), 1):
            device_name = filename.split('.')[0] if '.' in filename else filename
            print(f"{i}. {filename}:")
            print(f"   链接: /dev/{device_name}")
            print(f"   规则: {content.strip()}")
            print()
    
    input("\n按Enter返回主菜单...")

def uninstall_device_rule():
    """卸载设备udev规则的交互流程"""
    print_header()
    print("选择要卸载的设备规则:\n")
    
    rules_dir = "/etc/udev/rules.d/"
    rule_files = []
    
    try:
        # 查找所有.rules文件
        if os.path.exists(rules_dir):
            rule_files = [f for f in os.listdir(rules_dir) if f.endswith('.rules')]
    except Exception as e:
        print(f"读取规则目录时出错: {e}")
    
    if not rule_files:
        print("未找到任何已安装的设备规则")
        input("\n按Enter返回主菜单...")
        return
    
    # 列出所有规则文件
    for i, rule_file in enumerate(rule_files, 1):
        device_name = rule_file.split('.')[0] if '.' in rule_file else rule_file
        print(f"{i}. {rule_file} (/dev/{device_name})")
    
    print("0. 返回主菜单")
    
    choice = input("\n请输入要卸载的设备编号 [0-{}]: ".format(len(rule_files)))
    if choice == "0":
        return
    
    try:
        index = int(choice) - 1
        if 0 <= index < len(rule_files):
            rule_file = rule_files[index]
            rule_path = os.path.join(rules_dir, rule_file)
            device_name = rule_file.split('.')[0]
            
            confirm = input(f"\n确定要卸载 /dev/{device_name} ({rule_path}) 的规则吗? (y/n): ")
            
            if confirm.lower() == 'y':
                try:
                    try:
                        os.remove(rule_path)
                    except PermissionError:
                        print("需要管理员权限删除规则...")
                        subprocess.run(["sudo", "rm", rule_path], check=True)
                    
                    print("正在重新加载udev规则...")
                    subprocess.run(["sudo", "udevadm", "control", "--reload-rules"], check=True)
                    subprocess.run(["sudo", "udevadm", "trigger"], check=True)
                    print(f"\n已成功卸载 /dev/{device_name} 的规则")
                except Exception as e:
                    print(f"卸载规则时出错: {e}")
                    print("请尝试手动删除规则文件:")
                    print(f"sudo rm {rule_path}")
                    print("sudo udevadm control --reload-rules")
                    print("sudo udevadm trigger")
            else:
                print("已取消卸载操作")
        else:
            print("无效的选项!")
    except ValueError:
        print("请输入有效的数字!")
    
    input("\n按Enter返回主菜单...")

def show_help():
    """显示帮助信息"""
    print_header()
    print("使用指南:\n")
    print("1. 安装设备规则: 为各种传感器创建udev规则，使其具有固定名称")
    print("   - 输入设备描述名称和符号链接名")
    print("   - 按照提示插拔设备")
    print("   - 系统会自动检测到新设备并创建相应规则")
    print("   - 规则安装后，设备将始终使用固定链接名\n")
    
    print("2. 列出已安装规则: 查看系统中所有已安装的传感器规则\n")
    
    print("3. 卸载设备规则: 删除不再需要的设备规则\n")
    
    print("4. 使用固定设备名的好处:")
    print("   - 避免设备重启后端口名称变化")
    print("   - 使launch文件配置更简单（port:=/dev/laser）")
    print("   - 简化多设备系统管理\n")
    
    print("5. 常见设备类型示例:")
    print("   - 激光雷达: /dev/laser")
    print("   - 底盘串口: /dev/chassis")
    print("   - IMU传感器: /dev/imu")
    print("   - 编码器: /dev/encoder")
    print("   - 手柄控制器: /dev/joy")
    print("   - 摄像头: /dev/camera")
    print("   - GPS模块: /dev/gps\n")
    
    print("6. 自定义设备说明:")
    print("   - 您可以为任何串口设备创建自定义规则")
    print("   - 设备符号链接名只能包含字母、数字和下划线\n")
    
    print("提示: 可能需要管理员权限来安装或卸载规则")
    
    input("\n按Enter返回主菜单...")

def main_menu():
    """主菜单循环"""
    while True:
        print_header()
        print("主菜单:")
        print("1. 安装设备规则")
        print("2. 列出已安装规则")
        print("3. 卸载设备规则")
        print("4. 帮助")
        print("0. 退出程序")
        
        choice = input("\n请选择操作 [0-4]: ")
        
        if choice == "1":
            install_device_rule()
        elif choice == "2":
            list_installed_rules()
        elif choice == "3":
            uninstall_device_rule()
        elif choice == "4":
            show_help()
        elif choice == "0":
            print("\n谢谢使用，再见!")
            break
        else:
            print("无效选项! 请重试")
            input("按Enter继续...")

if __name__ == "__main__":
    try:
        main_menu()
    except KeyboardInterrupt:
        print("\n\n程序被用户中断")
        print("正在退出...")
    except Exception as e:
        print(f"\n程序发生错误: {e}")
        print("请尝试重新运行程序")
    finally:
        print("再见!")