
(cl:in-package :asdf)

(defsystem "lilac_fundamentals-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ExecutePlan" :depends-on ("_package_ExecutePlan"))
    (:file "_package_ExecutePlan" :depends-on ("_package"))
  ))