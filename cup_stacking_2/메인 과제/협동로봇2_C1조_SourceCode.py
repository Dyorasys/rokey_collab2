# 2025-02-28
# 컵 스택킹
# 스택킹 완료


# ------개선 사항------



# ------추가적으로 할 거-----
# 외부에서 충격주면 정리하는 트리거
# movesx 사용해서 감속 줄이기
# 한번에 여러개 잡아서 이동 거리 줄이기

# pick and place in 1 method. from pos_stack_1 to pos_stack_2 @20241104

import rclpy
import DR_init
import numpy as np

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 600, 600
VELOCITY2, ACC2 = 800, 800

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

OFF, ON = 0, 1


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("sorting", namespace=ROBOT_ID)

    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            release_compliance_ctrl,
            check_force_condition,
            task_compliance_ctrl,
            set_desired_force,
            set_tool,
            set_tcp,
            movej,
            movel,
            DR_FC_MOD_REL,
            DR_AXIS_Z,
            DR_BASE,
            trans,
            get_current_posx,
            get_current_posj,
            set_digital_output,
            wait,
            movesx,
            DR_MVS_VEL_NONE,
            set_stiffnessx,
            DR_BASE,
        )

        from DR_common2 import posx, posj

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return
    
    ####################################################
    # 함수들
    ####################################################

    # 힘 제어
    def force():
        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        set_desired_force(fd=[0, 0, -10, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        while not check_force_condition(DR_AXIS_Z, max=5):
            pass
        
        poss = get_current_posx()[0]
        release_compliance_ctrl()
        return poss



    # 그리퍼 해제
    def release():
        set_digital_output(2, ON)
        wait(0.5)
        set_digital_output(1, OFF)
        print('-re-')
        # wait_digital_input(2)

    # 그리퍼 그립
    def grip():
        set_digital_output(2, OFF)
        wait(0.5)
        set_digital_output(1, ON)
        print('-gr-')
        # wait_digital_input(1)

    # 그리퍼 그립
    def done():
        set_digital_output(1, ON)
        wait(0.5)
        set_digital_output(2, ON)
        print('-do-')
        # wait_digital_input(1)

    
    ############################################
    # 초기 값 설정
    ############################################

    # --------쌓여있는 좌표-----
    pos_stack_x = 351.949
    pos_stack_y = 147.924
    pos_stack_z = 185.0
    stack_dist = 10.0

    # --------타워 좌표--------
    # 잘되는 값
    # dx = 640.592
    # dy = 134.372
    # theta = -20.0 * np.pi/180.0

    # d = 189.981

    # 잘 되는 값 2
    dx = 640.592
    dy = 134.372
    theta = -50.0 * np.pi/180.0

    d = 159.981

    # 변환
    trans_matrix = np.array([[np.cos(theta), -np.sin(theta), dx],
                            [np.sin(theta), np.cos(theta), dy],
                            [0, 0, 1]])
    
    tp1 = np.array([d/np.sqrt(3), 0, 1])
    tp2 = np.array([-d/(2.0*np.sqrt(3)), d/2.0, 1])
    tp3 = np.array([-d/(2.0*np.sqrt(3)), -d/2.0, 1])

    tp1 = trans_matrix.dot(tp1.T)
    tp2 = trans_matrix.dot(tp2.T)
    tp3 = trans_matrix.dot(tp3.T)

    # 방향 선정
    point_list = [tp1, tp2, tp3]
    xlist = [tp1[0], tp2[0], tp3[0]]
    ylist = [tp1[1], tp2[1], tp3[1]]

    one = ylist.index(max(ylist))

    pos_tower_x1 = point_list[one][0]
    pos_tower_y1 = point_list[one][1]
    pos_tower_z1 = 77.311

    del point_list[one]
    del xlist[one]

    four = xlist.index(max(xlist))

    pos_tower_x4 = point_list[four][0]
    pos_tower_y4 = point_list[four][1]

    pos_tower_x6 = point_list[four-1][0]
    pos_tower_y6 = point_list[four-1][1]


    # 놓기 전에 얼마나 올라갈 지
    up = 10.0
    # 컵의 높이
    height = 105.0

    # --------나머지 타워 좌표 계산------
    pos_tower_x2 = (pos_tower_x1 + pos_tower_x4) / 2
    pos_tower_y2 = (pos_tower_y1 + pos_tower_y4) / 2

    pos_tower_x3 = (pos_tower_x1 + pos_tower_x6) / 2
    pos_tower_y3 = (pos_tower_y1 + pos_tower_y6) / 2

    pos_tower_x5 = (pos_tower_x4 + pos_tower_x6) / 2
    pos_tower_y5 = (pos_tower_y4 + pos_tower_y6) / 2

    # ---------------------------
    pos_tower_x7 = (pos_tower_x1 + pos_tower_x2 + pos_tower_x3) / 3
    pos_tower_y7 = (pos_tower_y1 + pos_tower_y2 + pos_tower_y3) / 3

    pos_tower_x8 = (pos_tower_x2 + pos_tower_x4 + pos_tower_x5) / 3
    pos_tower_y8 = (pos_tower_y2 + pos_tower_y4 + pos_tower_y5) / 3

    pos_tower_x9 = (pos_tower_x3 + pos_tower_x5 + pos_tower_x6) / 3
    pos_tower_y9 = (pos_tower_y3 + pos_tower_y5 + pos_tower_y6) / 3

    # ---------------------------
    pos_tower_x10 = (pos_tower_x7 + pos_tower_x8 + pos_tower_x9) / 3
    pos_tower_y10 = (pos_tower_y7 + pos_tower_y8 + pos_tower_y9) / 3

    # --------회전 각도--------
    ori_x = -90.0
    ori_y = -90.0
    ori_z_o = 270.0
    ori_z_e = 90.0

    # -------정리하기 위해 일시적으로 놓는 곳--------
    organize = posx([750.0, -100.0, pos_stack_z + 300.0, 180.0, -90.0, -90.0])
    organize_1 = posx([750.0, -100.0, pos_stack_z - stack_dist * 4 , 180.0, -90.0, -90.0])
    organize_2 = posx([750.0, -100.0, pos_stack_z - stack_dist, 180.0, -90.0, -90.0])
    organize_3 = posx([750.0, -100.0, pos_stack_z - stack_dist + 50.0, 180.0, -90.0, -90.0])

    ox = -120.0

    initial_pos = posx([pos_stack_x, pos_stack_y, pos_stack_z +50.0, ori_x, ori_y, ori_z_o])

    # 컵이 탐색 위치
    pos_stack_1 = posx([pos_stack_x, pos_stack_y, pos_stack_z - stack_dist * 0, ori_x, ori_y, ori_z_o])
    pos_stack_2 = posx([pos_stack_x, pos_stack_y, pos_stack_z - stack_dist * 1, ori_x, ori_y, ori_z_o])
    pos_stack_3 = posx([pos_stack_x, pos_stack_y, pos_stack_z - stack_dist * 2, ori_x, ori_y, ori_z_o])
    pos_stack_4 = posx([pos_stack_x, pos_stack_y, pos_stack_z - stack_dist * 3, ori_x, ori_y, ori_z_o])
    pos_stack_5 = posx([pos_stack_x, pos_stack_y, pos_stack_z - stack_dist * 4, ori_x, ori_y, ori_z_o])
    pos_stack_6 = posx([pos_stack_x, pos_stack_y, pos_stack_z - stack_dist * 5, ori_x, ori_y, ori_z_o])
    pos_stack_7 = posx([pos_stack_x, pos_stack_y, pos_stack_z - stack_dist * 6, ori_x, ori_y, ori_z_o])
    pos_stack_8 = posx([pos_stack_x, pos_stack_y, pos_stack_z - stack_dist * 7, ori_x, ori_y, ori_z_o])
    pos_stack_9 = posx([pos_stack_x, pos_stack_y, pos_stack_z - stack_dist * 8, ori_x, ori_y, ori_z_o])
    pos_stack_10 = posx([pos_stack_x, pos_stack_y, pos_stack_z - stack_dist * 9, ori_x, ori_y, ori_z_o])
    pos_stack_11 = posx([pos_stack_x, pos_stack_y, pos_stack_z - stack_dist * 10, ori_x, ori_y, ori_z_o])

    # 컵 탐색 위치
    pos_stack = [pos_stack_1,pos_stack_2,pos_stack_3,pos_stack_4,pos_stack_5,
                 pos_stack_6,pos_stack_7,pos_stack_8,pos_stack_9,pos_stack_10,
                 pos_stack_11]
                 

    #############################################################

    # 타워 설치 위치
    # _s는 정리용 위치

    # 1층
    pos_tower_1 = posx([pos_tower_x1 ,pos_tower_y1, pos_tower_z1, ox, ori_y, ori_z_o])
    pos_tower_4 = posx([pos_tower_x4 ,pos_tower_y4, pos_tower_z1, ox, ori_y, ori_z_o])
    pos_tower_6 = posx([pos_tower_x6 ,pos_tower_y6, pos_tower_z1, ox, ori_y, ori_z_o])
    pos_tower_6_s = posx([pos_tower_x6 ,pos_tower_y6, pos_tower_z1 + 80.0, ox, ori_y, ori_z_o])

    pos_tower_2 = posx([pos_tower_x2 ,pos_tower_y2, pos_tower_z1, ox, ori_y, ori_z_o])
    pos_tower_3 = posx([pos_tower_x3 ,pos_tower_y3, pos_tower_z1, ox, ori_y, ori_z_o])
    pos_tower_3_s = posx([pos_tower_x3 ,pos_tower_y3, pos_tower_z1 + 70.0, ox, ori_y, ori_z_o])
    pos_tower_5 = posx([pos_tower_x5 ,pos_tower_y5, pos_tower_z1, ox, ori_y, ori_z_o])
    pos_tower_5_s = posx([pos_tower_x5 ,pos_tower_y5, pos_tower_z1 + 70.0, ox, ori_y, ori_z_o])

    # 2층
    pos_tower_7 = posx([pos_tower_x7 ,pos_tower_y7, pos_tower_z1 + height, ox, ori_y, ori_z_o])
    pos_tower_8 = posx([pos_tower_x8 ,pos_tower_y8, pos_tower_z1 + height, ox, ori_y, ori_z_o])
    pos_tower_9 = posx([pos_tower_x9 ,pos_tower_y9, pos_tower_z1 + height, ox, ori_y, ori_z_o])
    pos_tower_9_s = posx([pos_tower_x9 ,pos_tower_y9, pos_tower_z1 + height * 2, ox, ori_y, ori_z_o])
    
    # 3층
    pos_tower_10 = posx([pos_tower_x10 ,pos_tower_y10, pos_tower_z1 + height * 2, ox, ori_y, ori_z_o])

    # 4층
    pos_tower_11 = posx([pos_tower_x10 ,pos_tower_y10, pos_tower_z1 + height * 3 - 120.0, ox, ori_y, ori_z_e])


    # 타워 놓일 위치 위쪽

    # 1층
    pos_tower_1_up = posx([pos_tower_x1 ,pos_tower_y1, pos_tower_z1 + up, ox, ori_y, ori_z_o])
    pos_tower_4_up = posx([pos_tower_x4 ,pos_tower_y4, pos_tower_z1 + 10.0 + up + 70.0, ox, ori_y, ori_z_o])
    pos_tower_6_up = posx([pos_tower_x6 ,pos_tower_y6, pos_tower_z1 + up, ox, ori_y, ori_z_o])

    pos_tower_2_up = posx([pos_tower_x2 ,pos_tower_y2, pos_tower_z1 + up + 70.0, ox, ori_y, ori_z_o])
    pos_tower_3_up = posx([pos_tower_x3 ,pos_tower_y3, pos_tower_z1 + up, ox, ori_y, ori_z_o])
    pos_tower_5_up = posx([pos_tower_x5 ,pos_tower_y5, pos_tower_z1 + up + 70.0, ox, ori_y, ori_z_o])

    # 2층
    pos_tower_7_up = posx([pos_tower_x7 ,pos_tower_y7, pos_tower_z1 + height + up + 100.0, ox, ori_y, ori_z_o])
    pos_tower_8_up = posx([pos_tower_x8 ,pos_tower_y8, pos_tower_z1 + height + up + 70.0, ox, ori_y, ori_z_o])
    pos_tower_9_up = posx([pos_tower_x9 ,pos_tower_y9, pos_tower_z1 + height + up, ox, ori_y, ori_z_o])
    
    # 3층
    pos_tower_10_up = posx([pos_tower_x10 ,pos_tower_y10, pos_tower_z1 + height * 2 + up, ox, ori_y, ori_z_o])

    # 4층
    pos_tower_11_up = posx([pos_tower_x10 ,pos_tower_y10, pos_tower_z1 + height * 3 + up + 20.0, ox, ori_y, ori_z_e])

    # 타워 놓일 위치
    pos_tower = [pos_tower_1, pos_tower_2, pos_tower_3, pos_tower_4,
                 pos_tower_5, pos_tower_6, pos_tower_7, pos_tower_8,
                 pos_tower_9, pos_tower_10, pos_tower_11]

    # 타워 놓기 전 위에 위치
    pos_tower_up = [pos_tower_1_up, pos_tower_2_up, pos_tower_3_up, pos_tower_4_up,
                 pos_tower_5_up, pos_tower_6_up, pos_tower_7_up, pos_tower_8_up,
                 pos_tower_9_up, pos_tower_10_up, pos_tower_11_up]


    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")

    ###########################################
    # 메인
    ###########################################

    while rclpy.ok():
        # 초기 위치로 이동
        # movej(JReady, vel=VELOCITY, acc=ACC)
        # 동작
        
        # stacking
        for i in range(len(pos_tower)):
            # 초기 위치로
            print(f'start {i+1}')
            movel(initial_pos, vel=VELOCITY2, acc=ACC2)
            release()
            wait(0.3)

            # 쌓여 있는 곳으로
            print('-------to stack-----')
            movel(pos_stack[i], vel=VELOCITY, acc=ACC)

            wait(0.3)

            # 내려가면서 잡기
            print('-----find------')
            force()
            print('-----grip------')
            wait(0.5)

            # 올라오기
            print('-----rise-------')
            t = trans(pos_stack[i], [0, 0, 150.0, 0, 0, 0], DR_BASE, DR_BASE)
            movel(t, vel=VELOCITY, acc=ACC)

            if i == 10:
                p = posj([-25.852, 25.93, 117.777, 69.336, 100.955, -235.122])
                movej(p, vel=VELOCITY, acc=ACC)
                break
                

            # 타워로 이동
            print('------to tower--------')
            movel(pos_tower_up[i],vel=VELOCITY, acc=ACC)
            movel(pos_tower[i], vel=VELOCITY, acc=ACC)
            
            # 내려놓기
            print('------drop---------')
            force()
            done()
            print('------done drop-------')

            t1 = trans(pos_tower[i],[0, 0, 150.0, 0, 0, 0],DR_BASE, DR_BASE)
            movel(t1, vel=VELOCITY2, acc=ACC2)

        # 4층
        # 타워로 이동
        print('------last--------')
        movel(pos_tower_up[10],vel=VELOCITY, acc=ACC)
        escape = posx([pos_tower_x10 ,pos_tower_y10, pos_tower_z1 + height * 3 - 120.0, ox, ori_y, ori_z_e])
        
        movel(pos_tower[10], vel=VELOCITY, acc=ACC)
        
        # 내려놓기
        force()
        print('------drop---------')
        done()
        print('------done drop-------')
        wait(1)

        t1 = trans(escape,[-20.0, -50.0, 0, 0, 0, 0],DR_BASE, DR_BASE)
        movel(t1, vel=100, acc=100)        



        wait(2)



        # -------------------정리-------------------
        # 처음 컵 잡기
        print('------start organize--------')
        done()
        print('-------to orient')
        movel(pos_tower_11, vel=VELOCITY, acc=ACC)
        wait(1)
        t = trans(pos_tower_11, [0, 0, 50.0, 0, 0, 0], DR_BASE, DR_BASE)
        movel(t, vel=VELOCITY, acc=ACC)
        wait(1)

        grip()

        wait(0.5)

        movel(pos_tower_11_up, vel=VELOCITY, acc=ACC)

        rotate = posj([-19.75, 21.117, 78.552, 89.814, 81.905, 0])

        movej(rotate, vel=VELOCITY,acc=ACC)

        movel(pos_tower_10_up, vel=VELOCITY, acc=ACC)
        wait(1)
        movel(pos_tower_10, vel=VELOCITY, acc=ACC)
        # 4개 하나로 organize 1
        print('strat 1')
        poslist1 = [pos_tower_10, pos_tower_9_s,pos_tower_9]
      
        movesx(poslist1, vel=[100, 30], acc=[200, 60], vel_opt=DR_MVS_VEL_NONE)
        pos_tower_6_new = posx([pos_tower_x6 ,pos_tower_y6, pos_tower_z1 + 30.0, ox, ori_y, ori_z_o])
        poslist12 = [pos_tower_9, pos_tower_6_s,pos_tower_6_new]
      
        movesx(poslist12, vel=[100, 30], acc=[200, 60], vel_opt=DR_MVS_VEL_NONE)

        wait(0.5)

        print('release')
        done()
        wait(0.5)
        movel(pos_tower_6, vel=VELOCITY, acc=ACC)
        grip()

        t =posx([564.002, -127.81, pos_tower_z1, ox, ori_y, ori_z_o])
        t1 = posx([750.0, -100.0, pos_tower_z1 , 180.0, -90.0, -90.0])

        poslist_n = [pos_tower_6, t, t1]

        movesx(poslist_n, vel=[100, 30], acc=[200, 60], vel_opt=DR_MVS_VEL_NONE)

        done()

        movel(organize, vel=VELOCITY, acc=ACC)

        # 2개 하나로 organize 2 (2개)
        print('start 2')
        movel(pos_tower_8_up, vel=VELOCITY, acc=ACC)
        release()
        movel(pos_tower_8, vel=VELOCITY, acc=ACC)
        force()
        
        poslist2 = [pos_tower_8, pos_tower_5_s, pos_tower_5]

        movesx(poslist2, vel=[100, 30], acc=[200, 60], vel_opt=DR_MVS_VEL_NONE)

        grip()

        
        t = trans(pos_tower_5, [0, 0, 300, 0, 0, 0], DR_BASE, DR_BASE)
        movel(t, vel=VELOCITY, acc=ACC)
        movel(organize, vel=VELOCITY, acc=ACC)
        movel(organize_2, vel=VELOCITY, acc=ACC)

        done()
        wait(0.5)

        movel(organize, vel=VELOCITY, acc=ACC)

        # 2개 하나로 organize 2 (2개)
        print('start 3')
        movel(pos_tower_7_up, vel=VELOCITY, acc=ACC)
        release()
        movel(pos_tower_7, vel=VELOCITY, acc=ACC)
        force()
        
        poslist3 = [pos_tower_7, pos_tower_3_s, pos_tower_3]

        movesx(poslist3, vel=[100, 30], acc=[200, 60], vel_opt=DR_MVS_VEL_NONE)

        grip()

        t = trans(pos_tower_3, [0, 0, 300, 0, 0, 0], DR_BASE, DR_BASE)
        movel(t, vel=VELOCITY, acc=ACC)
        movel(organize, vel=VELOCITY, acc=ACC)
        movel(organize_2, vel=VELOCITY, acc=ACC)

        done()
        wait(0.5)

        movel(organize, vel=VELOCITY, acc=ACC)

        # 전체 합치기 organize 3
        new = [pos_tower_4, pos_tower_2, pos_tower_1]
        new_up = [pos_tower_4_up, pos_tower_2_up, pos_tower_1_up]
        for i in range(len(new)):
            movel(new_up[i], vel=VELOCITY, acc=ACC)
            release()
            movel(new[i], vel=VELOCITY, acc=ACC)
            force()

            t = trans(new[i], [0, 0, 400, 0, 0, 0])
            movel(t, vel=VELOCITY, acc=ACC)
            movel(organize, vel=VELOCITY, acc=ACC)
            movel(organize_3, vel=VELOCITY, acc=ACC)

            done()

            movel(organize, vel=VELOCITY, acc=ACC)

        # 전체 이동
        done()

        movel(organize_2, vel=VELOCITY, acc=ACC)

        wait(1)

        # force()
        grip()
        movel(pos_stack_1, vel=VELOCITY, acc=ACC)

        done()

        wait(0.5)

        movel(initial_pos, vel=VELOCITY, acc=ACC)

        break



    rclpy.shutdown()


if __name__ == "__main__":
    main()