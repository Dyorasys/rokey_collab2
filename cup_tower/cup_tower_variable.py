# 2025-02-25
# 컵 이상하게 쌓기
# 완성본
# 4520.json이랑 같이 사용 - 이건 이름이 RG2
# 535.json이랑 같이 사용 - 이건 이름이 cup_tower


# ------개선 사항
# 컵이 조금씩 위치가 틀어짐
# 처음 컵이 쌓여있는 위치를 정확하게 고정할 필요
# 현재 z축에 대한 힘에대해서만 반응하는데 x,y 방향에 대한 힘을 사용하면
# 위치를 자동으로 수정이 가능하지 않을까????


# pick and place in 1 method. from pos_stack_1 to pos_stack_2 @20241104

import rclpy
import DR_init

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60

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
        
        poss = get_current_posx()
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
        # wait_digital_input(1)

    
    ############################################
    # 초기 값 설정
    ############################################
    # 초기 위치
    height = 77.0   # 컵 높이보다 짧음
    # JReady = [0, 0, 90, 0, 90, 0]

    # --------타워 좌표--------
    pos_tower_x = 549.362
    pos_tower_y = 53.859
    pos_tower_z = 500.0

    # --------쌓여있는 좌표-----
    pos_stack_x = 349.362
    pos_stack_y = 53.859
    pos_stack_z = 130.0

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
    pos_tower_e = posx([pos_tower_x, pos_tower_y, pos_tower_z, ori_x, ori_y, ori_z_e])
    
    # 컵이 처음 쌓여있는 위치
    pos_stack_1 = posx([pos_stack_x, pos_stack_y, pos_tower_z, ori_x, ori_y, ori_z_o])
    pos_stack_2 = posx([pos_stack_x, pos_stack_y, pos_stack_z, ori_x, ori_y, ori_z_o])


    # 각 컵별로 타워에 놓여질 위치
    pos_1 = posx([pos_tower_x, pos_tower_y, pos_tower_z-height*5, ori_x, ori_y, ori_z_o])
    pos_2 = posx([pos_tower_x, pos_tower_y, pos_tower_z-height*4, ori_x, ori_y, ori_z_e])
    pos_3 = posx([pos_tower_x, pos_tower_y, pos_tower_z-height*3, ori_x, ori_y, ori_z_o])
    pos_4 = posx([pos_tower_x, pos_tower_y, pos_tower_z-height*2, ori_x, ori_y, ori_z_e])
    pos_5 = posx([pos_tower_x, pos_tower_y, pos_tower_z, ori_x, ori_y, ori_z_o])

    pos = [pos_1,pos_2,pos_3,pos_4,pos_5]

    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")

    ###########################################
    # 메인
    ###########################################

    while rclpy.ok():
        # 초기 위치로 이동
        # movej(JReady, vel=VELOCITY, acc=ACC)
        # 동작

        for i in range(5):
            release()
            print(f'i : {i}')
            # 쌓여 있는 곳으로
            print('-------to orient')
            movel(pos_stack_1, vel=VELOCITY, acc=ACC)
            movel(pos_stack_2, vel=VELOCITY, acc=ACC)

            print('-----find------')
            force()
            print('-----grip------')
            grip()
            wait(0.5)

            movel(pos_stack_1,vel=VELOCITY, acc=ACC)

            movel(pos_tower, vel=VELOCITY, acc=ACC)
            
            print('-------ready to drop--------')
            movel(pos[i], vel=VELOCITY, acc=ACC)
            print('------drop---------')
            force()
            done()
            print('------done drop-------')



            if i%2 == 0:
                movel(pos_tower_o,vel=VELOCITY, acc=ACC)
            else:
                movel(pos_tower_e,vel=VELOCITY, acc=ACC)
                # 관절이 계속 한방향으로 회전하여 한계 걸림 방지
                movej(pos_tower_j, vel=VELOCITY, acc=ACC)
        break



    rclpy.shutdown()


if __name__ == "__main__":
    main()