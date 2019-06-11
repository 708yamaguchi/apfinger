;; list1 : proximity sensor value list 0.5cm off the plane
;; list2 : proximity sensor value list 1.2cm off the plane
(defun calib (list1 list2)
  (mapcar #'(lambda (x y) (/ (- 1367.0 604) (- x y))) list1 list2))
