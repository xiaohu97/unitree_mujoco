R1@51000 = logs/rsl_rl/ustc_humanoid_ultra_27dof_mimic_houtaitui_0808base/
           2026-08-27_16-16-07_r1_0808base_forceland/model_51000.pt

从 0808 (2026-08-08_18-06-58/model_49999.pt) 热启动。
配置 = 0808 的 16 项奖励 + 4 个终止项 + feet_contact_force_excess (阈值 2.0 体重, 权重 -20)。
关键: 不含 anchor_yaw 终止, 不含手臂奖励 -- 这两项是 0820 之后髋 yaw 偏转的来源。
plant 保持当前值: 辨识 URDF + USTCActuator 力矩-速度曲线 + armature 0.02 + hip_yaw kd 1.6 + 延迟 1-3。

离线评测 (scene_27dof_identified.xml, 100 seed, 复位 +-0.5, 指令延迟 3 步 = 15 ms):
  跌倒        3/100
  漂移中位    0.559
  触地中位    1.51   (体重倍数)
  触地峰值    2.31
  抬腿峰值    0.544  (参考 0.542)
  髋 yaw 相对参考偏差 (40 seed 中位):  左 +22.3°   右 -24.9°

对照:
  0725       跌倒 0  漂移 0.410 触地 2.15/7.49 抬腿 0.547  左 +11.7° 右 -20.6°
  0808 起点  跌倒 0  漂移 0.371 触地 2.12/7.75 抬腿 0.543  左 +12.2° 右 -16.2°
  J1@62000   跌倒 11 漂移 0.282 触地 1.94/3.61 抬腿 0.545  左 -23.5° 右 -70.8°

已知短板: 漂移 0.559 差于 0808 的 0.371。原因是 0808 的终止项里没有 anchor_pos_xy
(水平漂移的硬约束)。R2 正在训练中，就是加回这一项。

---------------------------------
[INFO] [1787840253.117565207]:  初始化程序完成，以下将发送电机的力控指令 
[INFO] [1787840254.118527446]:  EtherCAT overrun stats: count=12, avg=1.06283 ms, max=1.163 ms
[INFO] [1787840255.118696889]:  EtherCAT overrun stats: count=7, avg=1.21929 ms, max=2.084 ms
[INFO] [1787840256.119016269]:  EtherCAT overrun stats: count=20, avg=1.3853 ms, max=3.014 ms
[INFO] [1787840257.120036077]:  EtherCAT overrun stats: count=11, avg=1.29464 ms, max=2.503 ms
[INFO] [1787840258.121074195]:  EtherCAT overrun stats: count=7, avg=1.06114 ms, max=1.143 ms
[INFO] [1787840259.121142779]:  EtherCAT overrun stats: count=14, avg=1.10843 ms, max=1.807 ms
[INFO] [1787840260.121441886]:  EtherCAT overrun stats: count=8, avg=1.27388 ms, max=1.922 ms
[INFO] [1787840261.123275200]:  EtherCAT overrun stats: count=16, avg=1.13206 ms, max=1.895 ms
[INFO] [1787840262.124086112]:  EtherCAT overrun stats: count=11, avg=1.08609 ms, max=1.454 ms
[INFO] [1787840263.124932062]:  EtherCAT overrun stats: count=8, avg=1.07025 ms, max=1.219 ms
[INFO] [1787840264.125831967]:  EtherCAT overrun stats: count=5, avg=1.1292 ms, max=1.333 ms
[INFO] [1787840265.126296251]:  EtherCAT overrun stats: count=11, avg=1.04318 ms, max=1.258 ms
[INFO] [1787840266.126736355]:  EtherCAT overrun stats: count=9, avg=1.17411 ms, max=2.022 ms
[INFO] [1787840267.127406307]:  EtherCAT overrun stats: count=8, avg=1.07638 ms, max=1.194 ms
[INFO] [1787840268.128222390]:  EtherCAT overrun stats: count=5, avg=1.127 ms, max=1.359 ms
[INFO] [1787840269.128845063]:  EtherCAT overrun stats: count=11, avg=1.186 ms, max=1.887 ms
[INFO] [1787840270.129855160]:  EtherCAT overrun stats: count=6, avg=1.02667 ms, max=1.084 ms
[EtherCAT Error] Dropped packet (Bad WKC!)

[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Slave 1 lost
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Slave 2 lost
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Slave 3 lost
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Slave 4 lost
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Slave 5 lost
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Error count too high!
[EtherCAT Error] Logging error...
ESTOP. EtherCAT became degraded at Thu Aug 27 22:17:50 2026
.
[EtherCAT Error] Stopping RT process.
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Error count too high!
[EtherCAT Error] Logging error...
ESTOP. EtherCAT became degraded at Thu Aug 27 22:17:50 2026
.
[EtherCAT Error] Stopping RT process.
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Error count too high!
[EtherCAT Error] Logging error...
ESTOP. EtherCAT became degraded at Thu Aug 27 22:17:50 2026
.
[EtherCAT Error] Stopping RT process.
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Error count too high!
[EtherCAT Error] Logging error...
ESTOP. EtherCAT became degraded at Thu Aug 27 22:17:50 2026
.
[EtherCAT Error] Stopping RT process.
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Error count too high!
[EtherCAT Error] Logging error...
ESTOP. EtherCAT became degraded at Thu Aug 27 22:17:50 2026
.
[EtherCAT Error] Stopping RT process.
[INFO] [1787840270.755755831]:  左臂3号电机超过限定扭矩,当前电机的期望扭矩为:31.3805 
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Error count too high!
[EtherCAT Error] Logging error...
ESTOP. EtherCAT became degraded at Thu Aug 27 22:17:50 2026
.
[EtherCAT Error] Stopping RT process.
[INFO] [1787840270.775638916]:  左臂3号电机超过限定扭矩,当前电机的期望扭矩为:31.3805 
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Error count too high!
[EtherCAT Error] Logging error...
ESTOP. EtherCAT became degraded at Thu Aug 27 22:17:50 2026
.
[EtherCAT Error] Stopping RT process.
[INFO] [1787840270.787770794]:  左臂3号电机超过限定扭矩,当前电机的期望扭矩为:33.6635 
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Error count too high!
[EtherCAT Error] Logging error...
ESTOP. EtherCAT became degraded at Thu Aug 27 22:17:50 2026
.
[EtherCAT Error] Stopping RT process.
[INFO] [1787840270.799650296]:  左臂3号电机超过限定扭矩,当前电机的期望扭矩为:32.3489 
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Error count too high!
[EtherCAT Error] Logging error...
ESTOP. EtherCAT became degraded at Thu Aug 27 22:17:50 2026
.
[EtherCAT Error] Stopping RT process.
[INFO] [1787840270.807618443]:  左臂3号电机超过限定扭矩,当前电机的期望扭矩为:31.5497 
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Error count too high!
[EtherCAT Error] Logging error...
ESTOP. EtherCAT became degraded at Thu Aug 27 22:17:50 2026
.
[EtherCAT Error] Stopping RT process.
[INFO] [1787840270.819656588]:  左臂3号电机超过限定扭矩,当前电机的期望扭矩为:31.5497 
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Error count too high!
[EtherCAT Error] Logging error...
ESTOP. EtherCAT became degraded at Thu Aug 27 22:17:50 2026
.
[EtherCAT Error] Stopping RT process.
[INFO] [1787840270.843644352]:  左臂2号电机离线！ 
[INFO] [1787840270.843670510]:  左臂1号电机收到返回帧的次数为: 0 
[INFO] [1787840270.843686601]:  左臂2号电机收到返回帧的次数为: 0 
[INFO] [1787840270.843701461]:  左臂3号电机收到返回帧的次数为: 0 
[INFO] [1787840270.843715822]:  左臂4号电机收到返回帧的次数为: 0 
[INFO] [1787840270.843730419]:  左臂5号电机收到返回帧的次数为: 20 
[INFO] [1787840270.843744481]:  左臂6号电机收到返回帧的次数为: 20 
[INFO] [1787840270.843759275]:  左臂7号电机收到返回帧的次数为: 20 
[INFO] [1787840270.843780014]:  左臂3号电机超过限定扭矩,当前电机的期望扭矩为:30.3845 
[INFO] [1787840270.843795542]:  左臂3号电机离线！ 
[INFO] [1787840270.843809843]:  左臂1号电机收到返回帧的次数为: 0 
[INFO] [1787840270.843823831]:  左臂2号电机收到返回帧的次数为: 0 
[INFO] [1787840270.843838006]:  左臂3号电机收到返回帧的次数为: 0 
[INFO] [1787840270.843852115]:  左臂4号电机收到返回帧的次数为: 0 
[INFO] [1787840270.843866086]:  左臂5号电机收到返回帧的次数为: 20 
[INFO] [1787840270.843881990]:  左臂6号电机收到返回帧的次数为: 20 
[INFO] [1787840270.843895807]:  左臂7号电机收到返回帧的次数为: 20 
[INFO] [1787840270.843910205]:  左臂4号电机离线！ 
[INFO] [1787840270.843923786]:  左臂1号电机收到返回帧的次数为: 0 
[INFO] [1787840270.843937761]:  左臂2号电机收到返回帧的次数为: 0 
[INFO] [1787840270.843951721]:  左臂3号电机收到返回帧的次数为: 0 
[INFO] [1787840270.843965632]:  左臂4号电机收到返回帧的次数为: 0 
[INFO] [1787840270.843979377]:  左臂5号电机收到返回帧的次数为: 20 
[INFO] [1787840270.843993298]:  左臂6号电机收到返回帧的次数为: 20 
[INFO] [1787840270.844007225]:  左臂7号电机收到返回帧的次数为: 20 
[INFO] [1787840270.844023244]:  右臂2号电机离线！ 
[INFO] [1787840270.844038398]:  右臂1号电机收到返回帧的次数为: 0 
[INFO] [1787840270.844052299]:  右臂2号电机收到返回帧的次数为: 0 
[INFO] [1787840270.844066393]:  右臂3号电机收到返回帧的次数为: 0 
[INFO] [1787840270.844080551]:  右臂4号电机收到返回帧的次数为: 0 
[INFO] [1787840270.844094930]:  右臂5号电机收到返回帧的次数为: 20 
[INFO] [1787840270.844108638]:  右臂6号电机收到返回帧的次数为: 20 
[INFO] [1787840270.844122831]:  右臂7号电机收到返回帧的次数为: 20 
[INFO] [1787840270.844137177]:  右臂3号电机离线！ 
[INFO] [1787840270.844150976]:  右臂1号电机收到返回帧的次数为: 0 
[INFO] [1787840270.844164733]:  右臂2号电机收到返回帧的次数为: 0 
[INFO] [1787840270.844178859]:  右臂3号电机收到返回帧的次数为: 0 
[INFO] [1787840270.844193009]:  右臂4号电机收到返回帧的次数为: 0 
[INFO] [1787840270.844206642]:  右臂5号电机收到返回帧的次数为: 20 
[INFO] [1787840270.844220772]:  右臂6号电机收到返回帧的次数为: 20 
[INFO] [1787840270.844234824]:  右臂7号电机收到返回帧的次数为: 20 
[INFO] [1787840270.844248754]:  右臂4号电机离线！ 
[INFO] [1787840270.844262888]:  右臂1号电机收到返回帧的次数为: 0 
[INFO] [1787840270.844277192]:  右臂2号电机收到返回帧的次数为: 0 
[INFO] [1787840270.844293733]:  右臂3号电机收到返回帧的次数为: 0 
[INFO] [1787840270.844310686]:  右臂4号电机收到返回帧的次数为: 0 
[INFO] [1787840270.844325661]:  右臂5号电机收到返回帧的次数为: 20 
[INFO] [1787840270.844341794]:  右臂6号电机收到返回帧的次数为: 20 
[INFO] [1787840270.844356378]:  右臂7号电机收到返回帧的次数为: 20 
[INFO] [1787840270.844397426]:  发送左臂阻尼控制指令 
[INFO] [1787840270.844412828]:  发送右臂阻尼控制指令 
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Error count too high!
[EtherCAT Error] Logging error...
ESTOP. EtherCAT became degraded at Thu Aug 27 22:17:50 2026
.
[EtherCAT Error] Stopping RT process.
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Error count too high!
[EtherCAT Error] Logging error...
ESTOP. EtherCAT became degraded at Thu Aug 27 22:17:50 2026
.
[EtherCAT Error] Stopping RT process.
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Error count too high!
[EtherCAT Error] Logging error...
ESTOP. EtherCAT became degraded at Thu Aug 27 22:17:50 2026
.
[EtherCAT Error] Stopping RT process.
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Error count too high!
[EtherCAT Error] Logging error...
ESTOP. EtherCAT became degraded at Thu Aug 27 22:17:50 2026
.
[EtherCAT Error] Stopping RT process.
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Error count too high!
[EtherCAT Error] Logging error...
ESTOP. EtherCAT became degraded at Thu Aug 27 22:17:50 2026
.
[EtherCAT Error] Stopping RT process.
[EtherCAT Error] Dropped packet (Bad WKC!)


Cbth@bth-NUC13ANKi7:~/robotspace/src/humanoid-control-xy/humanoid_rl_controllers/script$ python3 sim2real_humanoidultra27dof_walk.py 
[INFO] [1787840250.351910]: Deployment ready: default=walk, policies=walk/stand/pick/houtaitui. Y enables control; X toggles stand/walk; LT+RIGHT plays pick; LT+DOWN plays houtaitui; A toggles stand left-arm tracking. LT+B stops inference with kd-only damping; LB+RB latches emergency damping. Standard absolute tilt limit=36.9 deg; Mimic reference tilt-error limit=30.0 deg.
[INFO] [1787840250.353142]: walk enabled=False fault=False damping=False fresh=True arm=n/a mimic=n/a cmd=[0.00 0.00 0.00] inference=0.000ms
[INFO] [1787840251.357550]: walk enabled=False fault=False damping=False fresh=True arm=n/a mimic=n/a cmd=[0.00 0.00 0.00] inference=0.000ms
[INFO] [1787840252.357594]: walk enabled=False fault=False damping=False fresh=True arm=n/a mimic=n/a cmd=[0.00 0.00 0.00] inference=0.000ms
[INFO] [1787840252.917081]: Policy control enabled: True (state=walk)
[INFO] [1787840253.362697]: walk enabled=True fault=False damping=False fresh=True arm=n/a mimic=n/a cmd=[0.00 0.00 -0.00] inference=0.562ms
[INFO] [1787840254.363212]: walk enabled=True fault=False damping=False fresh=True arm=n/a mimic=n/a cmd=[0.00 0.00 -0.00] inference=0.522ms
[INFO] [1787840255.367236]: walk enabled=True fault=False damping=False fresh=True arm=n/a mimic=n/a cmd=[0.00 0.00 -0.00] inference=0.620ms
[INFO] [1787840256.367830]: walk enabled=True fault=False damping=False fresh=True arm=n/a mimic=n/a cmd=[0.00 0.00 -0.00] inference=0.599ms
[INFO] [1787840257.372275]: walk enabled=True fault=False damping=False fresh=True arm=n/a mimic=n/a cmd=[0.00 0.00 -0.00] inference=0.557ms
[INFO] [1787840258.377706]: walk enabled=True fault=False damping=False fresh=True arm=n/a mimic=n/a cmd=[0.00 0.00 -0.00] inference=0.572ms
[INFO] [1787840259.378243]: walk enabled=True fault=False damping=False fresh=True arm=n/a mimic=n/a cmd=[0.00 0.00 -0.00] inference=0.451ms
[INFO] [1787840260.382533]: walk enabled=True fault=False damping=False fresh=True arm=n/a mimic=n/a cmd=[0.00 0.00 -0.00] inference=0.303ms
[INFO] [1787840261.382775]: walk enabled=True fault=False damping=False fresh=True arm=n/a mimic=n/a cmd=[0.00 0.00 -0.00] inference=0.426ms
[INFO] [1787840262.387964]: walk enabled=True fault=False damping=False fresh=True arm=n/a mimic=n/a cmd=[0.00 0.00 -0.00] inference=0.346ms
[INFO] [1787840263.393360]: walk enabled=True fault=False damping=False fresh=True arm=n/a mimic=n/a cmd=[0.00 0.00 -0.00] inference=0.321ms
[INFO] [1787840264.392933]: walk enabled=True fault=False damping=False fresh=True arm=n/a mimic=n/a cmd=[0.00 0.00 -0.00] inference=0.328ms
[INFO] [1787840265.398586]: walk enabled=True fault=False damping=False fresh=True arm=n/a mimic=n/a cmd=[0.00 0.00 -0.00] inference=0.542ms
[INFO] [1787840266.402860]: walk enabled=True fault=False damping=False fresh=True arm=n/a mimic=n/a cmd=[0.00 0.00 -0.00] inference=0.402ms
[INFO] [1787840267.407766]: walk enabled=True fault=False damping=False fresh=True arm=n/a mimic=n/a cmd=[0.00 0.00 -0.00] inference=0.390ms
[INFO] [1787840268.412229]: walk enabled=True fault=False damping=False fresh=True arm=n/a mimic=n/a cmd=[0.00 0.00 -0.00] inference=0.601ms
[INFO] [1787840268.932276]: Control state switched to: houtaitui
[INFO] [1787840268.933672]: Mimic houtaitui started: 941 frames at 50.0 Hz (18.82 s).
[INFO] [1787840269.413009]: houtaitui enabled=True fault=False damping=False fresh=True arm=n/a mimic=0/940 cmd=[0.00 0.00 0.00] inference=0.338ms
[INFO] [1787840270.412915]: houtaitui enabled=True fault=False damping=False fresh=True arm=n/a mimic=50/940 cmd=[0.00 0.00 0.00] inference=0.453ms
[INFO] [1787840271.412972]: houtaitui enabled=True fault=False damping=False fresh=True arm=n/a mimic=100/940 cmd=[0.00 0.00 0.00] inference=0.418ms
[INFO] [1787840272.418005]: houtaitui enabled=True fault=False damping=False fresh=True arm=n/a mimic=151/940 cmd=[0.00 0.00 0.00] inference=0.242ms
[INFO] [1787840273.423139]: houtaitui enabled=True fault=False damping=False fresh=True arm=n/a mimic=201/940 cmd=[0.00 0.00 0.00] inference=0.179ms
[INFO] [1787840274.423159]: houtaitui enabled=True fault=False damping=False fresh=True arm=n/a mimic=251/940 cmd=[0.00 0.00 0.00] inference=0.220ms
[INFO] [1787840275.422865]: houtaitui enabled=True fault=False damping=False fresh=True arm=n/a mimic=301/940 cmd=[0.00 0.00 0.00] inference=0.234ms
[INFO] [1787840276.423747]: houtaitui enabled=True fault=False damping=False fresh=True arm=n/a mimic=351/940 cmd=[0.00 0.00 0.00] inference=0.299ms
[INFO] [1787840277.427475]: houtaitui enabled=True fault=False damping=False fresh=True arm=n/a mimic=401/940 cmd=[0.00 0.00 0.00] inference=0.238ms
[INFO] [1787840278.432528]: houtaitui enabled=True fault=False damping=False fresh=True arm=n/a mimic=451/940 cmd=[0.00 0.00 0.00] inference=0.216ms
[ERROR] [1787840278.842057]: Mimic houtaitui reference tilt error 30.5 deg exceeded 30.0 deg: damping mode latched.
[INFO] [1787840279.437351]: houtaitui enabled=False fault=True damping=False fresh=True arm=n/a mimic=472/940 cmd=[0.00 0.00 0.00] inference=0.237ms
[INFO] [1787840280.437722]: houtaitui enabled=False fault=True damping=False fresh=True arm=n/a mimic=472/940 cmd=[0.00 0.00 0.00] inference=0.237ms
[INFO] [1787840281.442345]: houtaitui enabled=False fault=True damping=False fresh=True arm=n/a mimic=472/940 cmd=[0.00 0.00 0.00] inference=0.237ms
[INFO] [1787840282.442827]: houtaitui enabled=False fault=True damping=False fresh=True arm=n/a mimic=472/940 cmd=[0.00 0.00 0.00] inference=0.237ms
[INFO] [1787840283.443010]: houtaitui enabled=False fault=True damping=False fresh=True arm=n/a mimic=472/940 cmd=[0.00 0.00 0.00] inference=0.237ms
[INFO] [1787840284.447314]: houtaitui enabled=False fault=True damping=False fresh=True arm=n/a mimic=472/940 cmd=[0.00 0.00 0.00] inference=0.237ms
[INFO] [1787840285.447411]: houtaitui enabled=False fault=True damping=False fresh=True arm=n/a mimic=472/940 cmd=[0.00 0.00 0.00] inference=0.237ms
[INFO] [1787840286.452369]: houtaitui enabled=False fault=True damping=False fresh=True arm=n/a mimic=472/940 cmd=[0.00 0.00 0.00] inference=0.237ms
[INFO] [1787840287.457436]: houtaitui enabled=False fault=True damping=False fresh=False arm=n/a mimic=472/940 cmd=[0.00 0.00 0.00] inference=0.237ms
[INFO] [1787840288.457467]: houtaitui enabled=False fault=True damping=False fresh=False arm=n/a mimic=472/940 cmd=[0.00 0.






腿部所有电机均正常在线,执行位控初始化程序(本程序保持至少10s，输出转速为10rpm) 
---------------------------------
---------------------------------
 臂部所有电机均正常在线,执行位控初始化程序(本程序保持至少10s，输出转速为10rpm) 
---------------------------------
---------------------------------
 腰部所有电机均正常在线,执行位控初始化程序(本程序保持至少10s，输出转速为10rpm) 
---------------------------------
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Dropped packet (Bad WKC!)

[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Slave 1 lost
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Slave 2 lost
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Slave 3 lost
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Slave 4 lost
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Slave 5 lost
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Error count too high!
[EtherCAT Error] Logging error...
ESTOP. EtherCAT became degraded at Thu Aug 27 23:03:43 2026
.
[EtherCAT Error] Stopping RT process.
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Error count too high!
[EtherCAT Error] Logging error...
ESTOP. EtherCAT became degraded at Thu Aug 27 23:03:43 2026
.
[EtherCAT Error] Stopping RT process.
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Error count too high!
[EtherCAT Error] Logging error...
ESTOP. EtherCAT became degraded at Thu Aug 27 23:03:43 2026
.
[EtherCAT Error] Stopping RT process.
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Error count too high!
[EtherCAT Error] Logging error...
ESTOP. EtherCAT became degraded at Thu Aug 27 23:03:44 2026
.
[EtherCAT Error] Stopping RT process.
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Error count too high!
[EtherCAT Error] Logging error...
ESTOP. EtherCAT became degraded at Thu Aug 27 23:03:44 2026
.
[EtherCAT Error] Stopping RT process.
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Error count too high!
[EtherCAT Error] Logging error...
ESTOP. EtherCAT became degraded at Thu Aug 27 23:03:44 2026
.
[EtherCAT Error] Stopping RT process.
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Error count too high!
[EtherCAT Error] Logging error...
ESTOP. EtherCAT became degraded at Thu Aug 27 23:03:44 2026
.
[EtherCAT Error] Stopping RT process.
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Error count too high!
[EtherCAT Error] Logging error...
ESTOP. EtherCAT became degraded at Thu Aug 27 23:03:44 2026
.
[EtherCAT Error] Stopping RT process.
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Error count too high!
[EtherCAT Error] Logging error...
ESTOP. EtherCAT became degraded at Thu Aug 27 23:03:44 2026
.
[EtherCAT Error] Stopping RT process.
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Error count too high!
[EtherCAT Error] Logging error...
ESTOP. EtherCAT became degraded at Thu Aug 27 23:03:44 2026
.
[EtherCAT Error] Stopping RT process.
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Error count too high!
[EtherCAT Error] Logging error...
ESTOP. EtherCAT became degraded at Thu Aug 27 23:03:44 2026
.
[EtherCAT Error] Stopping RT process.
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Error count too high!
[EtherCAT Error] Logging error...
ESTOP. EtherCAT became degraded at Thu Aug 27 23:03:44 2026
.
[EtherCAT Error] Stopping RT process.
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Error count too high!
[EtherCAT Error] Logging error...
ESTOP. EtherCAT became degraded at Thu Aug 27 23:03:44 2026
.
[EtherCAT Error] Stopping RT process.
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Error count too high!
[EtherCAT Error] Logging error...
ESTOP. EtherCAT became degraded at Thu Aug 27 23:03:44 2026
.
[EtherCAT Error] Stopping RT process.
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Error count too high!
[EtherCAT Error] Logging error...
ESTOP. EtherCAT became degraded at Thu Aug 27 23:03:44 2026
.
[EtherCAT Error] Stopping RT process.
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Error count too high!
[EtherCAT Error] Logging error...
ESTOP. EtherCAT became degraded at Thu Aug 27 23:03:44 2026
.
[EtherCAT Error] Stopping RT process.
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Error count too high!
[EtherCAT Error] Logging error...
ESTOP. EtherCAT became degraded at Thu Aug 27 23:03:44 2026
.
[EtherCAT Error] Stopping RT process.
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Error count too high!
[EtherCAT Error] Logging error...
ESTOP. EtherCAT became degraded at Thu Aug 27 23:03:44 2026
.
[EtherCAT Error] Stopping RT process.
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Error count too high!
[EtherCAT Error] Logging error...
ESTOP. EtherCAT became degraded at Thu Aug 27 23:03:44 2026
.
[EtherCAT Error] Stopping RT process.
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Error count too high!
[EtherCAT Error] Logging error...
ESTOP. EtherCAT became degraded at Thu Aug 27 23:03:44 2026
.
[EtherCAT Error] Stopping RT process.
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Error count too high!
[EtherCAT Error] Logging error...
ESTOP. EtherCAT became degraded at Thu Aug 27 23:03:44 2026
.
[EtherCAT Error] Stopping RT process.
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Error count too high!
[EtherCAT Error] Logging error...
ESTOP. EtherCAT became degraded at Thu Aug 27 23:03:44 2026
.
[EtherCAT Error] Stopping RT process.
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Error count too high!
[EtherCAT Error] Logging error...
ESTOP. EtherCAT became degraded at Thu Aug 27 23:03:44 2026
.
[EtherCAT Error] Stopping RT process.
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Error count too high!
[EtherCAT Error] Logging error...
ESTOP. EtherCAT became degraded at Thu Aug 27 23:03:44 2026
.
[EtherCAT Error] Stopping RT process.
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Error count too high!
[EtherCAT Error] Logging error...
ESTOP. EtherCAT became degraded at Thu Aug 27 23:03:44 2026
.
[EtherCAT Error] Stopping RT process.
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Error count too high!
[EtherCAT Error] Logging error...
ESTOP. EtherCAT became degraded at Thu Aug 27 23:03:44 2026
.
[EtherCAT Error] Stopping RT process.
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Error count too high!
[EtherCAT Error] Logging error...
ESTOP. EtherCAT became degraded at Thu Aug 27 23:03:44 2026
.
[EtherCAT Error] Stopping RT process.
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Error count too high!
[EtherCAT Error] Logging error...
ESTOP. EtherCAT became degraded at Thu Aug 27 23:03:44 2026
.
[EtherCAT Error] Stopping RT process.
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Error count too high!
[EtherCAT Error] Logging error...
ESTOP. EtherCAT became degraded at Thu Aug 27 23:03:44 2026
.
[EtherCAT Error] Stopping RT process.
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Error count too high!
[EtherCAT Error] Logging error...
ESTOP. EtherCAT became degraded at Thu Aug 27 23:03:44 2026
.
[EtherCAT Error] Stopping RT process.
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Error count too high!
[EtherCAT Error] Logging error...
ESTOP. EtherCAT became degraded at Thu Aug 27 23:03:44 2026
.
[EtherCAT Error] Stopping RT process.
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Error count too high!
[EtherCAT Error] Logging error...
ESTOP. EtherCAT became degraded at Thu Aug 27 23:03:44 2026
.
[EtherCAT Error] Stopping RT process.
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Error count too high!
[EtherCAT Error] Logging error...
ESTOP. EtherCAT became degraded at Thu Aug 27 23:03:44 2026
.
[EtherCAT Error] Stopping RT process.
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Error count too high!
[EtherCAT Error] Logging error...
ESTOP. EtherCAT became degraded at Thu Aug 27 23:03:44 2026
.
[EtherCAT Error] Stopping RT process.
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Error count too high!
[EtherCAT Error] Logging error...
ESTOP. EtherCAT became degraded at Thu Aug 27 23:03:44 2026
.
[EtherCAT Error] Stopping RT process.
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Dropped packet (Bad WKC!)
[EtherCAT Error] Dropped packet (Bad WKC!)


