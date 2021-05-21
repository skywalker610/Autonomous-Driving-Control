
(cl:in-package :asdf)

(defsystem "hellocm_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "CM2Ext" :depends-on ("_package_CM2Ext"))
    (:file "_package_CM2Ext" :depends-on ("_package"))
    (:file "CM2Ext_vhcl" :depends-on ("_package_CM2Ext_vhcl"))
    (:file "_package_CM2Ext_vhcl" :depends-on ("_package"))
    (:file "Ext2CM" :depends-on ("_package_Ext2CM"))
    (:file "_package_Ext2CM" :depends-on ("_package"))
  ))