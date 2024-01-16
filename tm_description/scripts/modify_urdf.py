
import os
import shutil
import sys

import xml.etree.cElementTree as ET

import rospy

# from _modify_urdf import *
from _modify_urdf import modify_urdf
from _modify_urdf import urdf_DH_from_tm_DH
from _modify_urdf import xyzrpys_from_urdf_DH
from tm_msgs.srv import AskItem


def _gen_urdf():
    rospy.init_node('modify_urdf')

    ###############################################################################################
    # example: generate a new_model file (xxxooo.urdf), base on tm5s-norminal model
    # syntax : python3 modify_urdf.py original_model new_model
    # [key-in] original_model: tm5s  , [key-in] new_model: xxxooo
    # [key-in] shell cmd $ python3 modify_urdf.py tm5s xxxooo
    ###############################################################################################

    if len(sys.argv) < 3:
        print('Incorrect syntax! at least 2 parameters are required')
        print('You can try: python3 modify_urdf.py tm5s test')
        return

    original_model = sys.argv[1]
    new_model = sys.argv[2]
    specific_w = ''
    # specific keyword default
    overwrite = False
    nominal_model_restore = False
    tm5s_nominal_restore = False
    tm7s_nominal_restore = False
    tm12s_nominal_restore = False
    tm14s_nominal_restore = False
    tm25s_nominal_restore = False    
    tm_model = 'reference'
    ###############################################################################################
    # You can restore some nominal kinematic parameters by using specific keyword settings
    if len(sys.argv) == 4:
        specific_w = sys.argv[3].upper()
    if new_model == 'tm5s-nominal' or specific_w == '-K5S':
        tm_model = 'tm5s-nominal'
        nominal_model_restore = True
        tm5s_nominal_restore = True
    elif new_model == 'tm7s-nominal' or specific_w == '-K7S':
        tm_model = 'tm7s-nominal'
        nominal_model_restore = True
        tm7s_nominal_restore = True
    elif new_model == 'tm12s-nominal' or specific_w == '-K12S':
        tm_model = 'tm12s-nominal'
        nominal_model_restore = True
        tm12s_nominal_restore = True
    elif new_model == 'tm14s-nominal' or specific_w == '-K14S':
        tm_model = 'tm14s-nominal'
        nominal_model_restore = True
        tm14s_nominal_restore = True
    elif new_model == 'tm25s-nominal' or specific_w == '-K25S':
        tm_model = 'tm25s-nominal'
        nominal_model_restore = True
        tm25s_nominal_restore = True           
    else:
        nominal_model_restore = False
    if nominal_model_restore is True:
        message_s0 = 'Notice! You have chosen to restore a ' + tm_model + ' urdf model file'
        rospy.loginfo('%s!' % message_s0)
    if specific_w == '-OW':
        overwrite = True
        message_s1 = 'Notice!!! You have chosen to overwrite the original ' + tm_model + ' file'
        rospy.loginfo('%s!' % message_s1)
    ###############################################################################################

    rospy.wait_for_service('tm_driver/ask_item')
    ask_item = rospy.ServiceProxy('tm_driver/ask_item', AskItem)

    # Notice !!! You must have finished to run the driver to connect to youur TM Robot before.
    # [svr] (ask_item) -> id:dh (DHTable),id:dd (DeltaDH)
    res_dh = ask_item('dh', 'DHTable', 1.0)
    res_dd = ask_item('dd', 'DeltaDH', 1.0)

    if not res_dh.value.startswith('DHTable={') or not res_dh.value.endswith('}'):
        rospy.logerr('stop service, invalid parameters dh')
        return
    if not res_dd.value.startswith('DeltaDH={') or not res_dd.value.endswith('}'):
        rospy.logerr('stop service, invalid parameters delta_dh')
        return

    if not nominal_model_restore or overwrite:
        rospy.loginfo('loading the correction kinematics parameters from your TM Robot')
        if specific_w == '-VAL':
            rospy.loginfo(res_dh.value)
            rospy.loginfo(res_dd.value)

    dh_strs = res_dh.value[9:-1].split(',')
    dd_strs = res_dd.value[9:-1].split(',')

    ###############################################################################################
    # You can restore some nominal kinematic parameters by using specific keyword settings
    if nominal_model_restore is True:
        if tm5s_nominal_restore is True:
            rospy.loginfo('Restore with TM5S nominal kinematics parameters')
            res_dh = 'DHTable={0,-90,0,148.4,0,-360,360,-90,0,429,0,0,-360,360,0,0,386,0,0,-158,158,90,90,0,-147.8,0,-360,360,0,90,0,131.5,0,-360,360,0,0,0,134.95,0,-360,360}'
            res_dd = 'DeltaDH={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}'
        elif tm7s_nominal_restore is True:
            rospy.loginfo('Restore with TM7S nominal kinematics parameters')
            res_dh = 'DHTable={0,-90,0,148.4,0,-360,360,-90,0,329,0,0,-360,360,0,0,298,0,0,-152,152,90,90,0,-147.8,0,-360,360,0,90,0,131.5,0,-360,360,0,0,0,134.95,0,-360,360}'
            res_dd = 'DeltaDH={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}'
        elif tm12s_nominal_restore is True:
            rospy.loginfo('Restore with TM12S nominal kinematics parameters')
            res_dh = 'DHTable={0,-90,0,165.2,0,-360,360,-90,0,636.1,0,0,-360,360,0,0,532.4,0,0,-162,162,90,90,0,-181.8,0,-360,360,0,90,0,131.5,0,-360,360,0,0,0,134.95,0,-360,360}'
            res_dd = 'DeltaDH={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}'
        elif tm14s_nominal_restore is True:
            rospy.loginfo('Restore with TM14S nominal kinematics parameters')
            res_dh = 'DHTable={0,-90,0,165.2,0,-360,360,-90,0,536.1,0,0,-360,360,0,0,432.4,0,0,-159,159,90,90,0,-181.8,0,-360,360,0,90,0,131.5,0,-360,360,0,0,0,134.95,0,-360,360}'
            res_dd = 'DeltaDH={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}'
        elif tm25s_nominal_restore is True:
            rospy.loginfo('Restore with TM25S nominal kinematics parameters')
            res_dh = 'DHTable={0,-90,0,235.0,0,-360,360,-90,0,890.0,0,0,-360,360,90,90,0,-70.0,0,-166,166,0,-90,0,660.0,0,-360,360,0,90,0,170.2,0,-360,360,0,0,0,152.95,0,-360,360}'
            res_dd = 'DeltaDH={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}'
        else:
            # Example: TM5S nominal kinematics parameters
            rospy.loginfo('Restore with TM5S nominal kinematics parameters')
            res_dh = 'DHTable={0,-90,0,148.4,0,-360,360,-90,0,429,0,0,-360,360,0,0,386,0,0,-158,158,90,90,0,-147.8,0,-360,360,0,90,0,131.5,0,-360,360,0,0,0,134.95,0,-360,360}'
            res_dd = 'DeltaDH={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}'
        rospy.loginfo(res_dh)
        rospy.loginfo(res_dd)
        dh_strs = res_dh[9:-1].split(',')
        dd_strs = res_dd[9:-1].split(',')
    ###############################################################################################

    if len(dh_strs) != 42:
        rospy.logerr('stop service, invalid dh parameters')
        return
    if len(dd_strs) != 30:
        rospy.logerr('stop service, invalid delta_dh parameters')
        return

    dh = [float(i) for i in dh_strs]
    dd = [float(i) for i in dd_strs]

    # find urdf path
    curr_path = os.path.dirname(os.path.abspath(__file__))
    dirs = ['src', 'devel']
    ind = -1
    for d in dirs:
        ind = curr_path.find(d)
        if (ind != -1):
            break
    if (ind == -1):
        rospy.logerr('workspace directory not find')
        return
    src_path = curr_path[:ind] + 'src'
    urdf_path = ''
    for dirpath, dirnames, filenames in os.walk(src_path):
        if dirpath.endswith('tm_description'):
            urdf_path = dirpath + '/urdf'
            break
    if (urdf_path == ''):
        rospy.logerr('urdf directory not found')
        return

    urdf_name = '/' + original_model + '-nominal.urdf'
    new_urdf_name = '/' + new_model + '.urdf'
    if specific_w == '+M':
        new_urdf_name = '/macro.' + new_model + '.urdf'

    file_in = urdf_path + urdf_name
    file_out = urdf_path + new_urdf_name

    link_head = "<?xml version=\"1.0\" ?>\n"
    rospy.loginfo('[reference file path:] %s' % file_in)

    fr = open(file_in, 'r')
    link_data = fr.read()
    fr.close()

    root = ET.fromstring(link_data)

    udh = urdf_DH_from_tm_DH(dh, dd)
    xyzs, rpys = xyzrpys_from_urdf_DH(udh)
    modify_urdf(root, xyzs, rpys, udh)

    link_data = ET.tostring(root, encoding='UTF-8').decode('UTF-8')

    data_out = link_head + link_data

    file_save = ''
    if overwrite:
        file_save = file_in
        shutil.copyfile(file_in, file_out)
    else:
        file_save = file_out

    fw = open(file_save, 'w')
    fw.write(data_out)

    if overwrite:
        rospy.loginfo('File saved with new kinematic values')
        rospy.loginfo('[overwrite reference file path:] ' + str(file_in))
        rospy.loginfo('[new save file path:] ' + str(file_out))
    elif nominal_model_restore:
        rospy.loginfo('File restored with the nominal kinematic values')
        rospy.loginfo('[new save file path:] ' + str(file_save))
    else:
        rospy.loginfo('File saved with new kinematic values')
        rospy.loginfo('[new save file path:] ' + str(file_save))
    fw.close()


def main():
    try:
        _gen_urdf()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
