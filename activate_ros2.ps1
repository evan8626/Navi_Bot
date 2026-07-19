# Activate the real ROS 2 environment for Navi_Bot (dot-source this):
#
#     . .\activate_ros2.ps1
#
# Composes, in order:
#   1. the pixi conda env  (Python 3.12.3 + the conda-forge runtime libs
#      this ROS 2 build links against)
#   2. the ROS 2 install   (local_setup.ps1 -- 355 packages incl. rclpy, rviz2)
#   3. the colcon overlay  (navi_bot + navi_bot_interfaces, built into
#      C:\Users\evan8\navi_bot_ros2_ws)
#
# After activation:
#   ros2 launch navi_bot simulation.launch.py     # graph + RViz
#   ros2 launch navi_bot robot.launch.py          # graph only
#   ros2 node list / topic list / topic echo ...
#
# Remember: `ros2 launch/run` use the INSTALLED copies. After editing any
# node or launch file, rebuild:
#   cd C:\Users\evan8\navi_bot_ros2_ws
#   colcon build --paths "C:\Users\evan8\Desktop\Navi Bot" --packages-select navi_bot --merge-install
# (add `--packages-select navi_bot_interfaces` + MSVC vcvars when a .msg changes)
#
# NOTE: keep this file pure ASCII -- PowerShell 5.1 reads BOM-less scripts as
# CP-1252, and multi-byte characters can inject stray quote bytes.
# If the ros2-windows folder moves, update $Ros2Root below -- nothing else.

$Ros2Root = "C:\Users\evan8\Downloads\ros2-lyrical-2026-06-23-windows-AMD64.zip\ros2-windows"
$Workspace = "C:\Users\evan8\navi_bot_ros2_ws"

# Idempotence guard: the pixi hook wraps the PowerShell prompt function, and
# wrapping it a second time makes the prompt call itself -> StackOverflowException
# that kills the whole session. Only run the hook if this session isn't
# already inside the pixi env.
if (-not $env:PIXI_ENVIRONMENT_NAME) {
    & "$env:USERPROFILE\.pixi\bin\pixi.exe" shell-hook --manifest-path "$Ros2Root\pixi.toml" -s powershell |
        Out-String | Invoke-Expression
} else {
    Write-Host "(pixi env already active -- skipping shell hook)" -ForegroundColor DarkGray
}
$env:COLCON_PYTHON_EXECUTABLE = "$Ros2Root\.pixi\envs\default\python.exe"

# The pixi manifest writes this var in cmd syntax ("%cd%\...") which PowerShell
# never expands -- and %cd% is wrong unless activated from the ros2 root anyway.
# Without it, rviz2 dies with: could not find the Qt platform plugin "windows".
$env:QT_QPA_PLATFORM_PLUGIN_PATH = "$Ros2Root\.pixi\envs\default\Library\lib\qt6\plugins\platforms"

# THE viewport fix: this rviz build's Qt6 GL viewport breaks under Windows
# display scaling != 100% (this machine runs 125%) -- symptoms ranged from a
# black/transparent/flickering 3D canvas to the map rendering as a tiny dot.
# Disabling Qt's HiDPI scaling makes the scene render correctly.
$env:QT_ENABLE_HIGHDPI_SCALING = "0"
$env:QT_SCALE_FACTOR = "1"

# Deterministic GL for rviz: Mesa (opengl32.dll beside rviz2.exe) with the
# software llvmpipe driver. The "active samplers..." GLSL message it logs is
# non-fatal. Revert to hardware GL anytime by deleting opengl32.dll and
# libgallium_wgl.dll from <ros2>\lib\rviz2\.
$env:GALLIUM_DRIVER = "llvmpipe"

# DDS transport: UDP loopback only. FastDDS's shared-memory transport leaves
# zombie segment files in C:\ProgramData\eprosima whenever a node is hard-
# killed (dashboard test runs, Ctrl+C, probes) and enough litter eventually
# segfaults every rclpy process with 0xC0000005. UDP on localhost costs
# nothing at this graph's message sizes and is immune to kill-litter.
$env:FASTDDS_BUILTIN_TRANSPORTS = "UDPv4"

# Line-buffered node output: without this, Windows buffers each Python node's
# stdout and a hard kill (taskkill, dashboard stop) loses every log line.
$env:PYTHONUNBUFFERED = "1"

# rviz2 renders through Mesa software OpenGL (llvmpipe): Mesa's opengl32.dll +
# libgallium_wgl.dll sit beside lib\rviz2\rviz2.exe (app-dir DLL search order,
# affects rviz only). Reason: this laptop's Intel iGPU driver fails the Map
# shader's GLSL validation, and NVIDIA-rendered frames can't present across
# adapters (transparent canvas). llvmpipe sidesteps both. Remove those DLLs to
# go back to hardware GL.
$env:GALLIUM_DRIVER = "llvmpipe"
. "$Ros2Root\local_setup.ps1"
if (Test-Path "$Workspace\install\local_setup.ps1") {
    . "$Workspace\install\local_setup.ps1"
}

Write-Host "ROS 2 (lyrical) active. Try: ros2 launch navi_bot simulation.launch.py" -ForegroundColor Green