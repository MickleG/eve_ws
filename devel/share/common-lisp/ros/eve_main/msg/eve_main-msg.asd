
(cl:in-package :asdf)

(defsystem "eve_main-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "EndEffectorPosition" :depends-on ("_package_EndEffectorPosition"))
    (:file "_package_EndEffectorPosition" :depends-on ("_package"))
  ))