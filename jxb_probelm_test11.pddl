(define (problem jxb_move_problem) (:domain jxb_move)
(:objects 
    arm -robot
    base1 base2 base3 on_900_base2 on_900_base3 -location
    base1_fix_device base2_fix_device base3_fix_device -fix_device
)

;;; 5. 初始状态设置（示例值）
(:init
  ; 离散状态初始化
  (stopped_at arm base1)                    ; 机械臂初始位于base1
  (adjacent base1 on_900_base2)            ; 基座相邻关系
  (adjacent on_900_base2 base2)
  (adjacent base2 on_900_base3)
  (adjacent on_900_base3 base3)
  (in_comms)                   ; 初始在通信区间内
  (stopped arm)                     ; 初始状态为停止
  ; 连续变量初始化
  (time_passing)
  (have_fix_device base1 base1_fix_device)
  (have_fix_device base2 base2_fix_device)
  (have_fix_device base3 base3_fix_device)
  (= (current_time) 0)
  (=(vel_max) 0.5)  
  (=(acc_max) 0.2)
  (=(acc_rate)0.2) 
  (=(vel) 0)  
  (=(angle) 0) 
  (=(distance_acc) 0.625)        ; 时间起点为0秒
  (at 0 (in_comms)  )
  (at 660 (not(in_comms)  ))
  (at 690 (in_comms)  )
  (at 1200 (not(in_comms)  ))
  (at 1500 (in_comms)  )
  (at 2900 (not(in_comms)  ))

   (at 0 (not (occlusion_active base2)))
    (at 690  (occlusion_active base2))
    (at 720 (not (occlusion_active base2)))
    (at 1500  (occlusion_active base2))
    (at 1650 (not (occlusion_active base2)))
    (=(angle_dist base1 on_900_base2 ) 260)
    (=(angle_dist on_900_base2 base2 ) 100)
    (=(angle_dist base2 on_900_base3 ) 290)
    (=(angle_dist on_900_base3 base3 ) 100)
)

(:goal (and (stopped_at arm base3))
    ;todo: put the goal condition here
)
)


