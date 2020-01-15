#!/usr/bin/env roseus

(ros::roseus "dummy_proximity_publisher")
(ros::load-ros-manifest "apfinger")

(defun dummy-proximities-publish (&optional (val 1000))
  (ros::advertise "/proximity_sensors/left/all" apfinger::Proximities)
  (ros::advertise "/proximity_sensors/right/all" apfinger::Proximities)
  (ros::rate 10)
  (let ((value nil)
        (prox-left-msg (instance apfinger::Proximities :init))
        (prox-right-msg (instance apfinger::Proximities :init)))
    (do-until-key
     ;; (setq value (+ val (- (random 1000.0) (/ val 2)))) ;; set random value for each proximity sensor
     (setq value 1000) ;; set constant value for each proximity sensor
     (setq prox-left-list (make-list 36 :initial-element value))
     (setq prox-right-list (make-list 36 :initial-element value))
     (setq prox-left-list (coerce prox-left-list integer-vector))
     (setq prox-right-list (coerce prox-right-list integer-vector))
     (send prox-left-msg :proximities prox-left-list)
     (send prox-right-msg :proximities prox-right-list)
     (ros::publish "/proximity_sensors/left/all" prox-left-msg)
     (ros::publish "/proximity_sensors/right/all" prox-right-msg)
     (ros::sleep))))


(defun dummy-proximity-array-publish (&optional (val 1000))
  (ros::advertise "/proximity_sensor_topic1" apfinger::ProximityArray)
  (ros::advertise "/proximity_sensor_topic2" apfinger::ProximityArray)
  (ros::rate 10)
  (let ((value nil)
        (prox-left-msg (instance apfinger::ProximityArray :init))
        (prox-right-msg (instance apfinger::ProximityArray :init)))
    (do-until-key
     (setq value (round (+ val (- (random 1000.0) (/ val 2)))))
     (setq prox-left-list
           (make-list 3 :initial-element
                      (instance apfinger::Proximity :init :proximity value)))
     (setq prox-right-list
           (make-list 3 :initial-element
                      (instance apfinger::Proximity :init :proximity value)))
     (send prox-left-msg :proximities prox-left-list)
     (send prox-right-msg :proximities prox-right-list)
     (setq aaa prox-left-list)
     (setq bbb prox-left-msg)
     (ros::publish "/proximity_sensor_topic1" prox-left-msg)
     (ros::publish "/proximity_sensor_topic2" prox-right-msg)
     (ros::sleep))))
