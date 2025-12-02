;Header and description

(define (domain jxb_move)

;remove requirements that are not needed
(:requirements :strips :fluents :durative-actions :timed-initial-literals :typing :conditional-effects :negative-preconditions :duration-inequalities :equality :time)

  (:types 
      base on_900 temminate_loc normal_stopped_loc - location
      arm - robot
  )
  (:predicates 
      (adjacent ?l1 ?l2 - location)
      (stopped_at ?a - robot ?l - location)
      (in_comms) 
      (in_shade)
      (occlusion_active ?b - location)
      (stopped ?a - robot)
      (time_passing)
      (accelerating)   ; 加速状态
      (decelerating)   ; 减速状态
      (uniform_moving) 
      (have_fix_device ?l - location ?f -fix_device)
      (capture_at ?f -fix_device ?l - location)
      ; 匀速状态
  )
  (:functions 
      (vel) (vel_max) (acc_rate) (acc_max)
      (angle_dist ?l1 ?l2 - location)
      (current_time) (angle) (distance_acc) ; 添加声明
  )

  ;;; 修正后的processes
  (:process accelerate
    :parameters ()
    :precondition (and (in_comms) (not(in_shade)) (<(vel)(vel_max)))
    :effect (and
        (increase (vel) (* #t acc_rate))
        (increase (angle) (* #t (vel)))
        (increase (current_time) (* #t 1.0)) 
    )
  )

  (:process decelerate
    :parameters (?l1 ?l2 -location)
    :precondition (and
        (in_comms) (not(in_shade))
        ; 使用公式计算减速距离
        (> (angle) (- (angle_dist ?l1 ?l2) (distance_acc)))
        (< (angle) (angle_dist ?l1 ?l2))
    )
    :effect (and
        (increase (current_time) (* #t 1.0))
        (decrease (vel) (* #t acc_rate))  ; 添加减速效果
        (increase (angle) (* #t (vel)))
    )
  )
  (:process uniform_motion
     :parameters(?l1 ?l2 -location)
     :precondition(and
         (in_comms)
         (not(in_shade))
         (=(vel)(vel_max))
         (< (angle) (- (angle_dist ?l1 ?l2) (/ (* (vel) (vel)) (* 2 acc_rate)))))
     :effect (and
         (increase (current_time) (* #t 1.0))
         (increase (angle) (* #t vel_max)))
)

(:process update_time
  :parameters()
  :precondition(and(time_passing))
  :effect (increase (current_time) (* #t 1)) ; #t为时间增量，每秒增加1
)
(:process shading
    :parameters (?r - robot ?b - location)
    :precondition (and(in_comms)(occlusion_active ?b)(stopped ?r ))                 ; 处于通信区间内
    :effect   
      (increase (current_time) (* #t 1.0))  
)



  (:durative-action move
    :parameters (?a -robot ?from ?to -location )
    :duration (= ?duration (+
        (* 2 (/ (vel_max) (acc_max))) 
        (/ (- (angle_dist ?from ?to) 
              (/ (* (vel_max) (vel_max)) (acc_max)) 
           ) 
           (vel_max)
        )
    ))
    :condition (and 
        (at start (stopped_at ?a ?from))
        (over all (in_comms))  ; 修正谓词
        (over all (not (occlusion_active ?from)))                         
        (at start (adjacent ?from ?to))
    )
    :effect (and 
        (at start (not (stopped_at ?a ?from)))  ; 修正谓词
        (at end (stopped_at ?a ?to))
        (increase (current_time) (* #t 1.0))
        (at end (stopped ?a))
    )
  )

  ;;定义动作捕获
  (:durative-action capture
    :parameters (?f -fix_device ?l - location ?a -robot)
    :duration (= ?duration 100)
    :condition (and 
        (at start (stopped_at ?a ?l ))
        (over all (in_comms))
        (over all (not(occlusion_active ?l)))
        (at start (have_fix_device ?l  ?f ))
        (at start (not(capture_at ?f  ?l))) 
    )
    :effect (and 
        (at end (capture_at ?f ?l))
        (increase (current_time) (* #t 1.0))
        (at end (stopped ?a))
    )
  )

  ;;定义动作释放
  (:durative-action release
    :parameters (?f -fix_device ?l - location ?a -robot)
    :duration (= ?duration 100)
    :condition (and 
        (at start (stopped_at ?a ?l ))
        (at start (capture_at ?f ?l))
    )
    :effect (and 
        (at end (not(capture_at ?f ?l)))
        (increase (current_time) (* #t 1.0))
        (at end (stopped ?a))
        (at end (stopped_at ?a ?l))
    )
  )
)

