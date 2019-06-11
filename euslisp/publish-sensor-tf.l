#!/usr/bin/env roseus

(ros::load-ros-manifest "sensor_msgs")

(defun main ()
  ;; calculate coords fixed on each side of the finger
  (ros::roseus "publish_sensor_tf")
  (unless (boundp '*tfb*)
    (setq *tfb* (instance ros::transform-broadcaster :init)))
  (ros::rate 30)
  (let ((i 0) (relative-coords nil) (points nil) (sensor-coords))
    (do-until-key
     (ros::sleep)
     (dolist (l/r (list "/left" "/right"))
       (dolist (frame (list "/top" "/side0" "/side1" "/side2" "/side3"))
         (cond
          ((equal l/r "/left")
           (cond ((equal frame "/top")
                  (setq relative-coords (make-coords :pos #f(40 8 0))))
                 ((equal frame "/side0")
                  (setq relative-coords (make-coords :pos #f(13 20 1)))
                  (send relative-coords :rotate pi/2 :z)
                  (send relative-coords :rotate pi/2 :x))
                 ((equal frame "/side1")
                  (setq relative-coords (make-coords :pos #f(13 7 20)))
                  (send relative-coords :rotate -pi/2 :y)
                  (send relative-coords :rotate pi :x))
                 ((equal frame "/side2")
                  (setq relative-coords (make-coords :pos #f(13 -7 1)))
                  (send relative-coords :rotate -pi/2 :z)
                  (send relative-coords :rotate -pi/2 :x))
                 ((equal frame "/side3")
                  (setq relative-coords (make-coords :pos #f(13 7 -20)))
                  (send relative-coords :rotate pi/2 :y))))
          ((equal l/r "/right")
           (cond ((equal frame "/top")
                  (setq relative-coords (make-coords :pos #f(40 -8 0))))
                 ((equal frame "/side0")
                  (setq relative-coords (make-coords :pos #f(13 -20 1)))
                  (send relative-coords :rotate -pi/2 :z)
                  (send relative-coords :rotate -pi/2 :x))
                 ((equal frame "/side1")
                  (setq relative-coords (make-coords :pos #f(13 -7 20)))
                  (send relative-coords :rotate -pi/2 :y)
                  (send relative-coords :rotate pi :x))
                 ((equal frame "/side2")
                  (setq relative-coords (make-coords :pos #f(13 7 1)))
                  (send relative-coords :rotate pi/2 :z)
                  (send relative-coords :rotate pi/2 :x))
                 ((equal frame "/side3")
                  (setq relative-coords (make-coords :pos #f(13 -7 -20)))
                  (send relative-coords :rotate pi/2 :y))
                 )))
         ;; calculate position of points from coords of each side of finger
         (cond ((equal frame "/top")
                (setq points
                      (list (float-vector 0 -3.25 -8.05)
                            (float-vector 0 3.25 -8.05)
                            (float-vector 0 -3.25 8.05)
                            (float-vector 0 3.25 8.05))))
               (t
                (setq points
                      (list (float-vector 0 -3.25 -12.75)
                            (float-vector 0 3.25 -12.75)
                            (float-vector 0 -3.25 -4.25)
                            (float-vector 0 3.25 -4.25)
                            (float-vector 0 -3.25 4.25)
                            (float-vector 0 3.25 4.25)
                            (float-vector 0 -3.25 12.75)
                            (float-vector 0 3.25 12.75)))))
         (setq i 0)
         (dolist (point points)
           (setq sensor-coords (copy-object relative-coords))
           (send sensor-coords :translate point)
           (send *tfb* :send-transform sensor-coords (concatenate string (subseq l/r 1 2) "_gripper_finger_link") (format nil "/proximity_sensor_tf~A~A/~A" l/r frame i))
           (setq i (+ i 1))))
         ))
     ))