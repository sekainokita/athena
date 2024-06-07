
(cl:in-package :asdf)

(defsystem "xsens_mti_driver-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "XsStatusWord" :depends-on ("_package_XsStatusWord"))
    (:file "_package_XsStatusWord" :depends-on ("_package"))
  ))