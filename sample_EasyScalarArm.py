import dynamixel_classes_for_windows as dyna 
import kbhit
import time


try:
    dxl = dyna.Dynamixel("COM3",57600)
    kb = kbhit.KBHit()
    time.sleep(0.5)

    def fin():
        dxl.disable_torque(1)
        dxl.disable_torque(2)
        dxl.disable_torque(3)
        dxl.close_port()
        kb.set_normal_term()

    # ID 1 土台左右　
    min_1=-400
    max_1=1400
    # ID 2 先端上下
    min_2=-6000
    max_2=2000
    # ID 3 先端左右
    min_3=1400
    max_3=2750


    dxl.set_mode_ex_position(1)
    dxl.set_mode_ex_position(2)
    dxl.set_mode_ex_position(3)

    now_goal1=int(dxl.read_position(1))
    now_goal2=int(dxl.read_position(2))
    now_goal3=int(dxl.read_position(3))

    dxl.enable_torque(1)
    dxl.enable_torque(2)
    dxl.enable_torque(3)

    p_g1=0
    p_g2=0
    p_g3=0

    print("---------------------------------")
    print("     Dynamixel READY TO MOVE   ")
    print("---------------------------------")

    while 1:
        if kb.kbhit():
            c = ord(kb.getch())
            if c==75: #L arrow
                now_goal1=now_goal1+50
            elif c==77: #R arrow
                now_goal1=now_goal1-50
            elif c==72 :#UP
                now_goal2=now_goal2+300
            elif c==80 : #DOWN
                now_goal2=now_goal2-300
            elif c==122: #Z
                now_goal3=now_goal3-50
            elif c==120: #X
                now_goal3=now_goal3+50
            elif c==27: #ESC
                break

            now_goal1=max(min_1,min(now_goal1,max_1))
            now_goal2=max(min_2,min(now_goal2,max_2))
            now_goal3=max(min_3,min(now_goal3,max_3))

            if p_g1!=now_goal1:
                dxl.write_position(1,now_goal1) 
            if p_g2!=now_goal2:
                dxl.write_position(2,now_goal2) 
            if p_g3!=now_goal3:
                dxl.write_position(3,now_goal3) 

            p_g1=now_goal1
            p_g2=now_goal2
            p_g3=now_goal3

    fin()

except KeyboardInterrupt:
    fin()
