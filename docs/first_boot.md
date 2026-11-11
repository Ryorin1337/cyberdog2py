# 拿到真机后的初始化步骤

## 1. 开机与配网
1. 充满电，把狗趴放在平整的地面上，长按电源键开机。
2. 手机安装官方 App（以说明书或包装上的二维码为准），登录小米账号。
3. 配网：长按狗头上的触摸板，进入配网模式。App 会生成一个带 WiFi 信息的二维码，让狗的摄像头扫码。
   - **电脑和狗必须连在同一个 WiFi 下。**
   - 配网成功后，在 App 或路由器的后台查看狗的 IP 地址。
4. 在 App 里完成 OTA 升级。

## 2. 开启开发者权限（必需）
官方刷机文档写的是："默认情况下所有的调试接口都是封禁的，可以通过申请开发者权限来开启调试接口。"

在 App 里申请成为开发者，等待审核通过，SSH 才能登录。

## 3. 验证 SSH
- WiFi：`ssh mi@<狗的IP>`
- Type-C 数据线（插在中间充电口右侧的接口）：`ssh mi@192.168.55.1`
- 拆开调试盖板后用网线：`ssh mi@192.168.44.1`

默认密码是 `123`。官方文档只写了数据线和网线两种方式，WiFi 能否直接 SSH 需要在真机上确认。

## 4. 运行自检
```bash
pip install paramiko
set CYBERDOG_PASSWORD=123
python -m cyberdog2.doctor <狗的IP>
```
自检会依次输出：系统信息、ROS 环境脚本、motion_manager 进程、代理启动日志（其中包括继承到的 ROS_DOMAIN_ID 等变量）、命名空间、关键接口是否存在、电量。

### 常见失败与处理
| 现象 | 处理 |
|---|---|
| SSH 认证失败或连接被拒 | 开发者权限没有开通，或 IP 不对；也可以改用 Type-C 数据线连接 |
| `找不到进程 motion_manager` | 进程名可能不一样。先 SSH 上去执行 `ps aux \| grep -i motion` 查实际进程名，再传 `inherit_env_from="实际进程名"` |
| `没有权限读取 /proc/.../environ` | 狗的 ROS 进程以 root 身份运行。传 `inherit_env_from=None, env={"ROS_DOMAIN_ID": "..."}` 手动指定环境变量 |
| `NamespaceNotFound` | 代理看不到狗的 ROS 节点，通常是 DDS 域不一致或 ROS 服务还没启动完成 |
| 某个接口显示"缺失" | 说明这一版固件里接口名不同。把 `list_names()` 的输出发给我，我来修改 `constants.Iface` |
