
(cl:in-package :asdf)

(defsystem "racecarDepth-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "ObsPose" :depends-on ("_package_ObsPose"))
    (:file "_package_ObsPose" :depends-on ("_package"))
  ))