
(cl:in-package :asdf)

(defsystem "sp_pos-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "utmData" :depends-on ("_package_utmData"))
    (:file "_package_utmData" :depends-on ("_package"))
    (:file "latlonData" :depends-on ("_package_latlonData"))
    (:file "_package_latlonData" :depends-on ("_package"))
  ))