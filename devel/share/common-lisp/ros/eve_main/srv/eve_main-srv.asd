
(cl:in-package :asdf)

(defsystem "eve_main-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "GetPosition" :depends-on ("_package_GetPosition"))
    (:file "_package_GetPosition" :depends-on ("_package"))
    (:file "GoToPosition" :depends-on ("_package_GoToPosition"))
    (:file "_package_GoToPosition" :depends-on ("_package"))
    (:file "HomeY" :depends-on ("_package_HomeY"))
    (:file "_package_HomeY" :depends-on ("_package"))
  ))