;; Auto-generated. Do not edit!


(when (boundp 'mr_msgs::PlannerResult)
  (if (not (find-package "MR_MSGS"))
    (make-package "MR_MSGS"))
  (shadow 'PlannerResult (find-package "MR_MSGS")))
(unless (find-package "MR_MSGS::PLANNERRESULT")
  (make-package "MR_MSGS::PLANNERRESULT"))

(in-package "ROS")
;;//! \htmlinclude PlannerResult.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass mr_msgs::PlannerResult
  :super ros::object
  :slots (_header _algorithm _path_length _planning_time _num_waypoints _success ))

(defmethod mr_msgs::PlannerResult
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:algorithm __algorithm) "")
    ((:path_length __path_length) 0.0)
    ((:planning_time __planning_time) 0.0)
    ((:num_waypoints __num_waypoints) 0)
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _header __header)
   (setq _algorithm (string __algorithm))
   (setq _path_length (float __path_length))
   (setq _planning_time (float __planning_time))
   (setq _num_waypoints (round __num_waypoints))
   (setq _success __success)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:algorithm
   (&optional __algorithm)
   (if __algorithm (setq _algorithm __algorithm)) _algorithm)
  (:path_length
   (&optional __path_length)
   (if __path_length (setq _path_length __path_length)) _path_length)
  (:planning_time
   (&optional __planning_time)
   (if __planning_time (setq _planning_time __planning_time)) _planning_time)
  (:num_waypoints
   (&optional __num_waypoints)
   (if __num_waypoints (setq _num_waypoints __num_waypoints)) _num_waypoints)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _algorithm
    4 (length _algorithm)
    ;; float64 _path_length
    8
    ;; float64 _planning_time
    8
    ;; int32 _num_waypoints
    4
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _algorithm
       (write-long (length _algorithm) s) (princ _algorithm s)
     ;; float64 _path_length
       (sys::poke _path_length (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _planning_time
       (sys::poke _planning_time (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; int32 _num_waypoints
       (write-long _num_waypoints s)
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _algorithm
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _algorithm (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _path_length
     (setq _path_length (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _planning_time
     (setq _planning_time (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; int32 _num_waypoints
     (setq _num_waypoints (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get mr_msgs::PlannerResult :md5sum-) "ef19d2ad7038d14cfe1f591d32dcfcbf")
(setf (get mr_msgs::PlannerResult :datatype-) "mr_msgs/PlannerResult")
(setf (get mr_msgs::PlannerResult :definition-)
      "# Benchmark result for a single planning run.
Header header

# Algorithm identifier, e.g. \"astar\", \"bcd\".
string algorithm

# Total path length in meters.
float64 path_length

# Wall-clock planning time in seconds.
float64 planning_time

# Number of waypoints in the published path.
int32 num_waypoints

# True if the planner found a feasible path.
bool success

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :mr_msgs/PlannerResult "ef19d2ad7038d14cfe1f591d32dcfcbf")


