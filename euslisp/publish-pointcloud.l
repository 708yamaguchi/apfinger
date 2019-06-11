#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "fetcheus")
(ros::load-ros-manifest "sensor_msgs")
(ros::load-ros-manifest "APFinger")
(ros::roseus "publish_pointcloud")

(defclass publish-pointcloud
  :super propertied-object
  :slots (fingers-
          frame-list-
          pointcloud-table-
          ))

(defmethod publish-pointcloud
  (:init ()
    (when (not (boundp '*tfl*))
      (setq *tfl* (instance ros::transform-listener :init)))
    (setq fingers- (list "/left" "/right"))
    (setq frame-list- (list "/top" "/side0" "/side1" "/side2" "/side3"))
    (setq pointcloud-table- (make-hash-table :test #'equal))
    (dolist (l/r fingers-)
      (sethash l/r pointcloud-table- (make-hash-table :test #'equal))
      (dolist (frame frame-list-)
        (sethash frame (gethash l/r pointcloud-table-) nil)
        (ros::subscribe (concatenate string "/proximity_distance" l/r frame)
                        APFinger::ProximityDistances
                        #'send self :cb-pub-pointcloud l/r frame)
        (ros::advertise (concatenate string "/proximity_pointcloud" l/r frame)
                        sensor_msgs::PointCloud2 1)))
    ;; NOTE: contents of /proximity_pointcloud/all is different from all the other pointcloud topics.
    ;; TODO fix the above problem
    (ros::advertise "/proximity_pointcloud/left" sensor_msgs::PointCloud2 1)
    (ros::advertise "/proximity_pointcloud/right" sensor_msgs::PointCloud2 1)
    (ros::advertise "/proximity_pointcloud/all" sensor_msgs::PointCloud2 1))
  (:cb-pub-pointcloud (l/r frame msg)
    ;; publish pointcloud calculated from proximity sensor values on the finger
    (let ((points nil)
          (pc nil)
          (point-list nil)
          (count nil)
          (all-points nil)
          (left-points nil)
          (right-points nil)
          (left-pointclouds nil)
          (right-pointclouds nil)
          (pointclouds nil)
          (relative-coords nil)
          (distances (coerce (send (copy-object msg) :data) cons)))
      ;; calculate coords fixed on each side of the finger
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
      ;; Do not adopt distance when it is over 50mm, because of its reliability
      ;; Do not use *inf* instead of big number, because usage of *inf* disables calculation of coordinate transformation (siyou wo dousiyouka, kangaetyuu)
      ;; Do not use too big number when (< x 50), because it is regarded as inf in euslisp's float
      (setq distances (coerce (mapcar #'(lambda (x) (if (< x 50) x 99999)) distances) float-vector))
      (cond ((equal frame "/top")
             (setq points
                   (list (float-vector (elt distances 0) -3.25 -8.05)
                         (float-vector (elt distances 1) 3.25 -8.05)
                         (float-vector (elt distances 2) -3.25 8.05)
                         (float-vector (elt distances 3) 3.25 8.05))))
            (t
             (setq points
                   (list (float-vector (elt distances 0) -3.25 -12.75)
                         (float-vector (elt distances 1) 3.25 -12.75)
                         (float-vector (elt distances 2) -3.25 -4.25)
                         (float-vector (elt distances 3) 3.25 -4.25)
                         (float-vector (elt distances 4) -3.25 4.25)
                         (float-vector (elt distances 5) 3.25 4.25)
                         (float-vector (elt distances 6) -3.25 12.75)
                         (float-vector (elt distances 7) 3.25 12.75)))))
      ;; calculate position of pointclouds from l/r gripper coords
      (setq points (mapcar #'(lambda (x) (coerce (send (send relative-coords :copy-worldcoords) :transform-vector x) cons)) points))
      (sethash frame (gethash l/r pointcloud-table-) points)
      (setq points (make-array (list (length points) (length (first points)))
                               :initial-contents points
                               :element-type :float))
      (setq points (instance pointcloud
                             :init
                             :points points
                             ;; :colors nil)) ;; without color
                             :colors (make-list (array-dimension points 0) :initial-element #f(1 0 0)))) ;; with color
      ;; (setq pointclouds (make-ros-msg-from-eus-pointcloud points :with-color nil)) ;; without color
      (setq pointclouds (make-ros-msg-from-eus-pointcloud points :with-color :rgb)) ;; with color
      (send pointclouds :header (send msg :header))
      (send pointclouds :header :frame_id (concatenate string (subseq l/r 1 2) "_gripper_finger_link"))
      (send pointclouds :fields (subseq (send pointclouds :fields) 0 3))
      (ros::publish (concatenate string "/proximity_pointcloud" l/r frame) pointclouds)

      ;; Merge pointclouds of several sensors into one topic (left and right)
      (dolist (l/r fingers-)
        (dolist (frame frame-list-)
          (if (setq points (gethash frame (gethash l/r pointcloud-table-)))
            (progn
              (if (equal l/r "/left")
                (setq left-points (append left-points points)))
              (if (equal l/r "/right")
                (setq right-points (append right-points points)))))))
      ;; publish left pointcloud
      (setq left-points (make-array (list (length left-points) (length (first left-points)))
                               :initial-contents left-points
                               :element-type :float))
      ;; make pointcloud
      (setq left-points (instance pointcloud
                                  :init
                                  :points left-points
                                  ;; :colors nil)) ;; without color
                                  :colors (make-list (array-dimension left-points 0) :initial-element #f(1 0 0)))) ;; with color
      ;; (setq left-pointclouds (make-ros-msg-from-eus-pointcloud left-points :with-color nil)) ;; without color
      (setq left-pointclouds (make-ros-msg-from-eus-pointcloud left-points :with-color :rgb)) ;; with color
      (send left-pointclouds :header (send msg :header))
      (send left-pointclouds :header :frame_id "l_gripper_finger_link")
      (if (equal l/r "/left")
        (progn
          (send left-pointclouds :fields (subseq (send left-pointclouds :fields) 0 3))
          (ros::publish "proximity_pointcloud/left" left-pointclouds)))
      ;; publish right pointcloud
      (setq right-points (make-array (list (length right-points) (length (first right-points)))
                                    :initial-contents right-points
                                    :element-type :float))
      ;; make pointcloud
      (setq right-points (instance pointcloud
                                   :init
                                   :points right-points
                                   ;; :colors nil)) ;; without color
                                   :colors (make-list (array-dimension right-points 0) :initial-element #f(1 0 0)))) ;; with color
      ;; (setq right-pointclouds (make-ros-msg-from-eus-pointcloud right-points :with-color nil)) ;; without color
      (setq right-pointclouds (make-ros-msg-from-eus-pointcloud right-points :with-color :rgb)) ;; with color
      (send right-pointclouds :header (send msg :header))
      (send right-pointclouds :header :frame_id "r_gripper_finger_link")
      (if (equal l/r "/right")
        (progn
          (send right-pointclouds :fields (subseq (send right-pointclouds :fields) 0 3))
          (ros::publish "proximity_pointcloud/right" right-pointclouds)))

      ;; publish /proximity_pointcloud/all which contains pointclouds from tf of each proximity sensor
      (setq count 0)
      (dolist (distance (coerce distances cons))
        (setq pc (instance pointcloud :init))
        ;; publish multiple points from one proximity sensor
        (setq point-list nil)
        (let ((grid-list
               (list -1.0 -0.875 -0.75 -0.625 -0.5 -0.375 -0.25 -0.125 0.0 0.125 0.25 0.375 0.5 0.625 0.75 0.875 1.0)
               ;; (list -2.0 -1.75 -1.5 -1.25 -1.0 -0.75 -0.5 -0.25 0.0 0.25 0.5 0.75 1.0 1.25 1.5 1.75 2.0)
               ;; (list -4.0 -3.5 -3.0 -2.5 -2.0 -1.5 -1.0 -0.5 0.0 0.5 1.0 1.5 2.0 2.5 3.0 3.5 4.0)
               ))
          (dolist (i grid-list)
            (dolist (j grid-list)
              (setq point-list (append point-list (list (list distance i j)))))))
        (setq point-list (apply #'matrix point-list))
        (send pc :points point-list)
        (ros::publish "proximity_pointcloud/all"
                      (make-ros-msg-from-eus-pointcloud pc :frame (format nil "/proximity_sensor_tf~A~A/~A" l/r frame count)))
        (setq count (+ 1 count)))
      ))
  )


(defun init ()
  (when (not (boundp '*pp*))
    (setq *pp* (instance publish-pointcloud :init))))

(defun main ()
  (init)
  (ros::rate 10)
  (while t
    (ros::spin-once)
    (ros::sleep)))
