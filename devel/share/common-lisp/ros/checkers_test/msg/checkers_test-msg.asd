
(cl:in-package :asdf)

(defsystem "checkers_test-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "floats" :depends-on ("_package_floats"))
    (:file "_package_floats" :depends-on ("_package"))
  ))