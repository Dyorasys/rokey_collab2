# 2025-02-25
# 컵 스택킹


# ------개선 사항------
# 미세 조정

# pick and place in 1 method. from pos_stack_1 to pos_stack_2 @20241104

import rclpy
import DR_init

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 80, 80

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
    # 초기 위치
    height = 80.0   # 컵 높이보다 짧음
    # JReady = [0, 0, 90, 0, 90, 0]

    # --------타워 좌표--------
    pos_tower_x = 549.362
    pos_tower_y = 53.859
    pos_tower_z = 450.0

    # --------쌓여있는 좌표-----
    pos_stack_x = 349.362
    pos_stack_y = 53.859
    pos_stack_z = 150.0

    # --------회전 각도--------
    ori_x = -90.0
    ori_y = -90.0
    ori_z_o = 270.0
    ori_z_e = 90.0

    # 타워 위치
    pos_tower = posx([pos_tower_x, pos_tower_y, pos_tower_z, ori_x, ori_y, ori_z_o])
    # posx, movel만 쓰면 6번 관절이 한 방향으로만 회전하여 limit에 걸림
    # 이를 방지하기 위한 posj
    pos_tower_j = posj([-28.595, 38.794, 44.158, 93.508, 118.110, 7.992])
    # 홀수번째는 그대로, 짝수번째는 6번 관절이 180도 회전
    # 컵을 쌓고 위로 올릴 때 회전하면서 타워를 쓰러트리는걸 방지
    pos_tower_o = posx([pos_tower_x, pos_tower_y, pos_tower_z, ori_x, ori_y, ori_z_o])
    # pos_tower_e = posx([pos_tower_x, pos_tower_y, pos_tower_z, ori_x, ori_y, ori_z_e])
    
    # 컵이 처음 쌓여있는 위치
    pos_stack_1 = posx([pos_stack_x, pos_stack_y, pos_tower_z, ori_x, ori_y, ori_z_o])
    pos_stack_2 = posx([pos_stack_x, pos_stack_y, pos_stack_z, ori_x, ori_y, ori_z_o])


    # 각 컵별로 타워에 놓여질 위치
    pos_1 = posx([pos_tower_x, pos_tower_y, pos_tower_z-height*4, ori_x, ori_y, ori_z_o])
    pos_1_up = posx([pos_tower_x, pos_tower_y, pos_tower_z-height*3, ori_x, ori_y, ori_z_o])

    pos_2 = posx([pos_tower_x-80.0, pos_tower_y, pos_tower_z-height*4, ori_x, ori_y, ori_z_o])
    pos_2_up = posx([pos_tower_x-80.0, pos_tower_y, pos_tower_z-height*3, ori_x, ori_y, ori_z_o])
    pos_2_prime = posx([pos_tower_x-80.0, pos_tower_y, pos_tower_z-height*3-10.0, ori_x, ori_y, ori_z_o])

    pos_3 = posx([pos_tower_x+80.0, pos_tower_y, pos_tower_z-height*4, ori_x, ori_y, ori_z_o])
    pos_3_up = posx([pos_tower_x+80.0, pos_tower_y, pos_tower_z-height*3, ori_x, ori_y, ori_z_o])
    pos_3_prime = posx([pos_tower_x+80.0, pos_tower_y, pos_tower_z-height*4+25.0, ori_x, ori_y, ori_z_o])

    pos_4 = posx([pos_tower_x-40.0, pos_tower_y, pos_tower_z-height*3, ori_x, ori_y, ori_z_o])
    pos_4_up = posx([pos_tower_x-40.0, pos_tower_y, pos_tower_z-height*2, ori_x, ori_y, ori_z_o])
    pos_4_prime = posx([pos_tower_x-60.0, pos_tower_y, pos_tower_z-height*3+25.0, ori_x, ori_y, ori_z_o])

    pos_5 = posx([pos_tower_x+40.0, pos_tower_y, pos_tower_z-height*3, ori_x, ori_y, ori_z_o])
    pos_5_up = posx([pos_tower_x+40.0, pos_tower_y, pos_tower_z-height*2, ori_x, ori_y, ori_z_o])

    pos_6 = posx([pos_tower_x, pos_tower_y, pos_tower_z-height*2, ori_x, ori_y, ori_z_o])
    pos_6_up = posx([pos_tower_x, pos_tower_y, pos_tower_z-height, ori_x, ori_y, ori_z_o])
    pos_6_prime = posx([pos_tower_x-10.0, pos_tower_y, pos_tower_z-height*2, ori_x, ori_y, ori_z_o])

    pos_oragnize = posx([pos_tower_x, pos_tower_y, pos_tower_z-height*1, ori_x, ori_y, ori_z_o])

    pos = [pos_1,pos_2,pos_3,pos_4,pos_5, pos_6]
    pos_up = [pos_1_up,pos_2_up,pos_3_up,pos_4_up,pos_5_up,pos_6_up,]

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
        for i in range(len(pos)):
            release()
            print(f'i : {i}')
            # 쌓여 있는 곳으로
            print('-------to orient')
            movel(pos_stack_1, vel=VELOCITY, acc=ACC)
            movel(pos_stack_2, vel=VELOCITY, acc=ACC)

            print('-----find------')
            force()
            print('-----grip------')
            wait(0.5)

            movel(pos_stack_1,vel=VELOCITY, acc=ACC)

            movel(pos_tower, vel=VELOCITY, acc=ACC)
            
            print('-------ready to drop--------')
            movel(pos_up[i], vel=VELOCITY, acc=ACC)
            movel(pos[i], vel=VELOCITY, acc=ACC)
            print('------drop---------')
            force()
            done()
            print('------done drop-------')

            movel(pos_up[i], vel=VELOCITY, acc=ACC)
            movel(pos_tower_o,vel=VELOCITY, acc=ACC)


        # 정리
        # 처음 컵 잡기
        print('------start organize--------')
        release()
        print('-------to orient')
        movel(pos_stack_1, vel=VELOCITY, acc=ACC)
        movel(pos_stack_2, vel=VELOCITY, acc=ACC)

        print('-----find------')
        force()
        print('-----grip------')
        wait(0.5)

        # 3개 하나로
        movel(pos_stack_1, vel=VELOCITY, acc=ACC)
        movel(pos_oragnize, vel=VELOCITY, acc=ACC)
        movel(pos_6, vel=VELOCITY, acc=ACC)
        print('pos_6')

        print('strat 1')
        poslist1 = [pos_6, pos_6_prime, pos_4_prime, pos_2]

        # task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])        
        movesx(poslist1, vel=[100, 30], acc=[200, 60], vel_opt=DR_MVS_VEL_NONE)
        # elease_compliance_ctrl()
        print('release')
        done()

        movel(pos_2_up, vel=VELOCITY, acc=ACC)

        # 2개 하나로
        movel(pos_oragnize, vel=VELOCITY, acc=ACC)
        movel(pos_5_up, vel=VELOCITY, acc=ACC)
        movel(pos_5, vel=VELOCITY, acc=ACC)
        print('start 2')
        poslist2 = [pos_5, pos_3_prime, pos_3]

        # task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])

        movesx(poslist2, vel=[100, 30], acc=[200, 60], vel_opt=DR_MVS_VEL_NONE)

        # release_compliance_ctrl()

        done()

        movel(pos_3, vel=VELOCITY, acc=ACC)

        # 전체 합치기
        # 2 + 4
        release()
        force()
        grip()
        movel(pos_oragnize, vel=VELOCITY, acc=ACC)
        movel(pos_2_up, vel=VELOCITY, acc=ACC)
        force()
        done()

        movel(pos_2_up, vel=VELOCITY, acc=ACC)

        release()

        # 1 + 6
        movel(pos_oragnize, vel=VELOCITY, acc=ACC)
        movel(pos_1, vel=VELOCITY, acc=ACC)
        force()

        movel(pos_oragnize, vel=VELOCITY, acc=ACC)
        movel(pos_2_prime, vel=VELOCITY, acc=ACC)
        poss = force()
        
        new_poss = [pos_stack_x, pos_stack_y, pos_tower_z, ori_x, ori_y, ori_z_o]

        new_poss[2] = poss[2]


        # 전체 이동
        movel(new_poss, vel=VELOCITY, acc=ACC)

        done()

        movel(pos_stack_1, vel=VELOCITY, acc=ACC)
        


        break



    rclpy.shutdown()


if __name__ == "__main__":
    main()
