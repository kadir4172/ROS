;; Auto-generated. Do not edit!


(when (boundp 'a3_help::RequestGoal)
  (if (not (find-package "A3_HELP"))
    (make-package "A3_HELP"))
  (shadow 'RequestGoal (find-package "A3_HELP")))
(unless (find-package "A3_HELP::REQUESTGOAL")
  (make-package "A3_HELP::REQUESTGOAL"))
(unless (find-package "A3_HELP::REQUESTGOALREQUEST")
  (make-package "A3_HELP::REQUESTGOALREQUEST"))
(unless (find-package "A3_HELP::REQUESTGOALRESPONSE")
  (make-package "A3_HELP::REQUESTGOALRESPONSE"))

(in-package "ROS")





(defclass a3_help::RequestGoalRequest
  :super ros::object
  :slots (_x _y ))

(defmethod a3_help::RequestGoalRequest
  (:init
   (&key
    ((:x __x) 0)
    ((:y __y) 0)
    )
   (send-super :init)
   (setq _x (round __x))
   (setq _y (round __y))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:serialization-length
   ()
   (+
    ;; int64 _x
    8
    ;; int64 _y
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _x
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _x (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _x) (= (length (_x . bv)) 2)) ;; bignum
              (write-long (ash (elt (_x . bv) 0) 0) s)
              (write-long (ash (elt (_x . bv) 1) -1) s))
             ((and (class _x) (= (length (_x . bv)) 1)) ;; big1
              (write-long (elt (_x . bv) 0) s)
              (write-long (if (>= _x 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _x s)(write-long (if (>= _x 0) 0 #xffffffff) s)))
     ;; int64 _y
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _y (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _y) (= (length (_y . bv)) 2)) ;; bignum
              (write-long (ash (elt (_y . bv) 0) 0) s)
              (write-long (ash (elt (_y . bv) 1) -1) s))
             ((and (class _y) (= (length (_y . bv)) 1)) ;; big1
              (write-long (elt (_y . bv) 0) s)
              (write-long (if (>= _y 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _y s)(write-long (if (>= _y 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _x
#+(or :alpha :irix6 :x86_64)
      (setf _x (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _x (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _y
#+(or :alpha :irix6 :x86_64)
      (setf _y (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _y (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(defclass a3_help::RequestGoalResponse
  :super ros::object
  :slots (_ack ))

(defmethod a3_help::RequestGoalResponse
  (:init
   (&key
    ((:ack __ack) nil)
    )
   (send-super :init)
   (setq _ack __ack)
   self)
  (:ack
   (&optional __ack)
   (if __ack (setq _ack __ack)) _ack)
  (:serialization-length
   ()
   (+
    ;; bool _ack
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _ack
       (if _ack (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _ack
     (setq _ack (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass a3_help::RequestGoal
  :super ros::object
  :slots ())

(setf (get a3_help::RequestGoal :md5sum-) "33e40041c161ed97cccea8af6f436cfd")
(setf (get a3_help::RequestGoal :datatype-) "a3_help/RequestGoal")
(setf (get a3_help::RequestGoal :request) a3_help::RequestGoalRequest)
(setf (get a3_help::RequestGoal :response) a3_help::RequestGoalResponse)

(defmethod a3_help::RequestGoalRequest
  (:response () (instance a3_help::RequestGoalResponse :init)))

(setf (get a3_help::RequestGoalRequest :md5sum-) "33e40041c161ed97cccea8af6f436cfd")
(setf (get a3_help::RequestGoalRequest :datatype-) "a3_help/RequestGoalRequest")
(setf (get a3_help::RequestGoalRequest :definition-)
      "int64 x
int64 y
---
bool ack

")

(setf (get a3_help::RequestGoalResponse :md5sum-) "33e40041c161ed97cccea8af6f436cfd")
(setf (get a3_help::RequestGoalResponse :datatype-) "a3_help/RequestGoalResponse")
(setf (get a3_help::RequestGoalResponse :definition-)
      "int64 x
int64 y
---
bool ack

")



(provide :a3_help/RequestGoal "33e40041c161ed97cccea8af6f436cfd")


