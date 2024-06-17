#!/usr/bin/env python3

import numpy as np
import cv2 as cv
import os
from datetime import datetime
import glob #for data search
import time


def feed_Checkerboard_photos(img):
    # used by fetching_from_camera()
    # used by fetching_from_file()

    # with given checker board img and known checker board size, provide its point locations in world frame and image frame
    # the capture boolean and accepted frames are returned as well
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    # checkerboard corner size
    h = 6
    w = 5
    objp = 0
    corners2 = 0
    checker_size = 28.6 # in mm
    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, (h,w), None)
    # If found, add object points, image points (after refining them)
    if ret == True:
        # termination criteria
        criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        objp = np.zeros((w*h,3), np.float32)
        objp[:,:2] = np.mgrid[0:h,0:w].T.reshape(-1,2)*checker_size 
     
        corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        
        # Draw and display the corners
        #print(objp,corners2)
        cv.drawChessboardCorners(img, (h,w), corners2, ret)
    #cv.imshow('img', img)
        
    return ret, objp, corners2, img

def fetching_from_camera(data_path):
    # used by extrinsic_confirmation()
    
    # Arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.
    total_img = 0

    cap = cv.VideoCapture(2,cv.CAP_V4L2)
    # cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    # width = 1920
    # height = 1080
    # cap.set(cv.CAP_PROP_FRAME_WIDTH, width)
    # cap.set(cv.CAP_PROP_FRAME_HEIGHT, height)
    if not cap.isOpened():
        print("Cannot open camera")
        exit()
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break

        processed_frame = frame.copy()
        found_checkerboard, objp, corners, img = feed_Checkerboard_photos(processed_frame)
        cv.imshow('Press "s" to save this frame "q" to proceed', img)

        # termination choices
        keyPressed = cv.waitKey(1)
        if keyPressed == ord('q'):
            break
        if keyPressed == ord('s'):
            if not os.path.exists(data_path):
                os.makedirs(data_path)
            total_img = total_img + 1
            now = datetime.now()
            dt_string = now.strftime("%Y-%m-%d %H:%M:%S")
            save_path = os.path.join(data_path, dt_string+'.png')
            cv.imwrite(save_path,frame)
            objpoints.append(objp)
            imgpoints.append(corners)
    
    print('image points of ', np.shape(imgpoints), ' with total of ', total_img,' images collected')
    cap.release()
    cv.destroyAllWindows()
    return objpoints, imgpoints, frame


def fetching_from_file(im_path):
    # used by extrinsic_confirmation()
    
    # Arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.
    num_valid_img = 0
    total_img = 0
    frame = []

    #print(im_path + '/*.png')
    images = glob.glob(im_path + '/*.png', recursive = False)

    for im in images:
        total_img = total_img + 1
        frame = cv.imread(im)
        found_checkerboard, objp, corners, _ = feed_Checkerboard_photos(frame)
        if found_checkerboard:
            num_valid_img = num_valid_img + 1
            objpoints.append(objp)
            imgpoints.append(corners)

    print(num_valid_img, ' out of ', total_img,' images were valid')
    return objpoints, imgpoints, frame
    

def draw(img, centers, imgpts, radius_coordinate):
    # used by extrinsic_confirmation()
    
    # draw the world frame origin and an projected location of the bowl
    # draw axis: BGR - xyz
    center = np.int0(centers[0].ravel())
    img = cv.line(img, center, np.int0(imgpts[0].ravel()), (255,0,0), 5)
    img = cv.line(img, center, np.int0(imgpts[1].ravel()), (0,255,0), 5)
    img = cv.line(img, center, np.int0(imgpts[2].ravel()), (0,0,255), 5)

    # draw a circle with the inner bowl radius
    radius = np.int0(radius_coordinate[0][0])
    radius = radius-center
    img = cv.circle(img, center, radius[0], (0,0,0), 3)
    
    return img


def intrinsic_cam_para():
    # public
    
    # load saved intrinsic parameters
    fname = "camera_calib_intrinsic.npy"
    mtx_path = os.path.abspath(os.path.join(os.path.abspath(__file__),
                                            os.pardir, 'calibrationData',fname))
    with open(mtx_path, 'rb') as file:
        mtx = np.load(file)
        dist = np.load(file)
    return mtx, dist


def imag_undistort(frame, mtx, dist):
    # public

    # uses intrinsic parameters to undistort a frame
    mtx_opt, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (640, 480), 1, (640, 480))
    frame_undist = cv.undistort(frame, mtx, dist, None, mtx_opt)
    return frame_undist


def excaliboard_worldframe_relocation(degree):
    # used by excaliboard_worldframe_points()
    
    # relocate the origin and orientation of the "excaliboard" into one of the following angles:
    # [0, 60, 120, 180, 240] in degrees
    o = [2,1]  # origin coordinate from bottom left
    #degree = 60  # angle of rotation
    checker_size = 30  # in mm
    h = 4
    w = 5

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    coord_x_dir = np.mgrid[0-o[0] : w-o[0], 0-o[1] : h-o[1]]
    coord_y_dir = np.empty((2,h,w), np.float32)
    coord_y_dir[0,:,:] = coord_x_dir[0,:,:].T
    coord_y_dir[1,:,:] = coord_x_dir[1,:,:].T
    im_points = coord_y_dir.T.reshape(-1,2)*checker_size

    # apply rotation around z-axis
    theta = np.radians(degree)
    c, s = np.cos(theta), np.sin(theta)
    R = np.array(((c, -s), (s, c)))
    im_points_transformed = np.dot(R, im_points.T).T

    # (n,2) to (n, 3) with zeros at z
    objp = np.zeros((w*h,3), np.float32)
    objp[:,:2] = np.round(im_points_transformed, decimals = 3)
    return objp


def feed_excaliboard_photo(img):
    # used by excaliboard_worldframe_points()
    
    # obtain the pixcels coordinates of the excaliboard
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    # checkerboard corner size
    h = 4
    w = 5
    objp = 0
    corners2 = 0
    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, (h,w), None)
    # If found, add object points, image points (after refining them)
    if ret == True:
        # termination criteria
        criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        
        # Draw and display the corners
        #print(objp,corners2)
        cv.drawChessboardCorners(img, (h,w), corners2, ret)
    #cv.imshow('img', img)
        
    return ret, corners2, img


def excaliboard_worldframe_points(data_path, degree):
    # used by extrinsic_confirmation()
    # used by excaliboard_reprojection_error()

    # provide the world frame and image frame points on the excaliboard
    # pre-define outputs
    objpoints = np.empty((0,1,3),float)
    imgpoints = np.empty((0,1,2),float)

    # turn on camera using openCV
    cap = cv.VideoCapture(4,cv.CAP_V4L2)
    if not cap.isOpened():
        print("Cannot open camera")
        exit()

    print('        collecting extrinsic parameters from the excaliboard')
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break

        #obtaint intrinsic camera parameters
        mtx, dist = intrinsic_cam_para()
        
        #undistort the frame
        frame = imag_undistort(frame, mtx, dist)

        processed_frame = frame.copy()
        found_checkerboard, corners, img = feed_excaliboard_photo(processed_frame)
        objp = excaliboard_worldframe_relocation(degree)  # excaliboard rotation in degrees
        
        cv.imshow('Press "s" to save this frame "q" to proceed with just data', img)

        # termination choices
        keyPressed = cv.waitKey(1)
        if keyPressed == ord('q'):
            objpoints = np.append(objpoints, np.reshape(np.array(objp), (-1, 1, 3)) ,axis = 0)
            imgpoints = np.append(imgpoints, np.array(corners), axis = 0)
            break
        if keyPressed == ord('s'):
            objpoints = np.append(objpoints, np.reshape(np.array(objp), (-1, 1, 3)) ,axis = 0)
            imgpoints = np.append(imgpoints, np.array(corners), axis = 0)
            if not os.path.exists(data_path):
                os.makedirs(data_path)
            now = datetime.now()
            dt_string = now.strftime("%Y-%m-%d_%H-%M-%S")
            save_path = os.path.join(data_path, 'Calibrated_at_'+dt_string+'.png')
            #cv.imwrite(save_path,frame)
            if not cv.imwrite(save_path, frame):
                raise Exception("Could not write image")
            time.sleep(3)
            break
    cap.release()
    cv.destroyAllWindows()
    return np.round(objpoints, decimals = 3), np.round(imgpoints, decimals = 3)
                                                       

def corrected_center():
    # public

    #load saved calibrated center information
    fname = "excaliboard_center_correction.npy"
    file_path = os.path.join(os.getcwd(), 'calibrationData/')
    center_true = []
    
    if os.path.exists(file_path+fname):
        with open(file_path+fname, 'rb') as file:
            center_true = np.load(file)
            print('Loaded True center of the bowl in pixel: \n', center_true)
    else:
        center_true = [320, 240]
        print('No true center of the bowl identified, run "excaliboard_center_correction()" to obtain')
    return center_true


def excaliboard_center_correction(imgpoints_old, objpoints_old):
    # used by extrinsic_confirmation()
    
    # bring the excaliboard in other tilting angles positions to the calculated center of the bowl
    
    # calculate the geometric center of the thumper bowl based on the location of the pegs
    fname = "excaliboard_center_correction.npy"
    file_path = os.path.join(os.getcwd(), 'calibrationData/')
    center_true = []
    
    # first check if correction has been done
    print('***start excaliboard center correction')
    if not os.path.exists(file_path+fname):
        centers = np.zeros((2,2),float)
        print('Place the excaliboard at 0 degree location and press "q" ')
        time.sleep(2)
        objpoints, imgpoints = excaliboard_worldframe_points(file_path, 0)
        isorig = np.all(objpoints == np.array([[0,0,0]]), axis = 2)
        centers[0,:] = imgpoints[isorig][0]

        print('Place the excaliboard at 180 degree location and press "q" ')
        time.sleep(2)
        objpoints, imgpoints = excaliboard_worldframe_points(file_path, 180)
        isorig = np.all(objpoints == np.array([[0,0,0]]), axis = 2)
        centers[1,:] = imgpoints[isorig][0]

        center_true = np.average(centers,axis = 0)
        # save the intrinsic calibrated parameters   
        with open(file_path+fname, 'wb') as file:
            np.save(file, center_true)
            print('Saved True center of the bowl in pixel: \n', center_true)
    else:
        # load the saved center of the bowl
        center_true = corrected_center()

    isorig = np.all(objpoints_old == np.array([[0,0,0]]), axis = 2)
    center_curr = imgpoints_old[isorig][0]

    diff = center_true - center_curr
    imgpoints_new = np.copy(imgpoints_old)
    imgpoints_new[:,0,0] = imgpoints_old[:,0,0] + diff[0]
    imgpoints_new[:,0,1] = imgpoints_old[:,0,1] + diff[1]

    return imgpoints_new

    
def extrinsic_confirmation(exp_path):
    # public
    
    # Main function to operate the extrinsic parameter calculations
    # calculate and inspect extrinsic parameter, save calibrated data if desirable

    #obtaint intrinsic camera parameters
    mtx, dist = intrinsic_cam_para()

    # PnP with known camera mtx and dist to get extrinsic parameter of fixed camera
    sq_obj, sq_img = excaliboard_worldframe_points(exp_path, 0)
    sq_img = excaliboard_center_correction(sq_img, sq_obj)
    images = glob.glob(exp_path + '/*.png', recursive = False)
    excaliboard_original = cv.imread(images[0])
    
    #print(np.shape(sq_obj),np.shape(sq_img))
    #print(sq_obj,'\n', sq_img)
    retval, r_co_rod, p_co = cv.solvePnP(sq_obj,sq_img, mtx, dist)
    r_co, jacobian = cv.Rodrigues(r_co_rod)
    r_co__rodd, jacobian = cv.Rodrigues(r_co)
    print('translation', p_co)
    print('rotation', r_co)

    # project world frame dimension onto the image frame
    bowl_diameter = 175.88 # mm
    axis_wd = np.float32([[60,0,0], [0,60,0], [0,0,60]]).reshape(-1,3) # in mm world frame
    orig_wd = np.float32([[0,0,0]])
    radius_wd = np.float32([bowl_diameter/2,0,0])
    axis_im, jacobian = cv.projectPoints(axis_wd, r_co_rod, p_co, mtx, dist)
    orig_im, jacobian = cv.projectPoints(orig_wd, r_co_rod, p_co, mtx, dist)
    radius_im, jacobian = cv.projectPoints(radius_wd, r_co_rod, p_co, mtx, dist)
    
    img = draw(excaliboard_original,orig_im,axis_im, radius_im)

    print('press "s" to confirm and save the location of the experimental setup')
    
    cv.imshow('Press "s" to save current extrinsic para "q" to abort', img)
    keyPressed = cv.waitKey(0)
    if keyPressed == ord('q'):
        cv.destroyAllWindows()
    if keyPressed == ord('s'):
        # save the intrinsic calibrated parameters
        fname = "camera_calib_extrinsic.npy"
        mtx_path = os.path.join(os.getcwd(), 'calibrationData/')
        with open(mtx_path+fname, 'wb') as file:
            np.save(file, r_co_rod)
            np.save(file, p_co)
            print('camera extrinsic parameters saved')
        cv.destroyAllWindows()


def extrinsic_cam_para():
    # public
    
    # load saved extrinsic parameters
    fname = "camera_calib_extrinsic.npy"
    mtx_path = os.path.join(os.getcwd(), 'calibrationData/')
    with open(mtx_path+fname, 'rb') as file:
        r_co_rod = np.load(file)
        p_co = np.load(file)
    return r_co_rod, p_co

        
def imageframe_to_worldframe(point_im):
    # public
    
    # input 2D uv pixel coordinate shape = (2, npoints)
    # output 3D world frame points in mm shape = (3,npoints)
    
    mtx, dist = intrinsic_cam_para()
    r_co_rod, p_co = extrinsic_cam_para()

    r_co, jacobian = cv.Rodrigues(r_co_rod)

##    print('camera matrix: \n', mtx, '\ndistortion: \n', dist,
##          '\nr_co_rodrigues: \n', r_co_rod,
##          '\nr_co: \n', r_co, '\ntranslation: \n', p_co)

    
    r_oc = np.linalg.inv(r_co)
    mtx_inv = np.linalg.inv(mtx)

    p_co_im = np.dot(mtx, p_co)
    s = p_co_im[2,0]
    
    _, npoints = np.shape(point_im)
    uv_im = np.vstack((point_im, np.ones((1,npoints))))
    point_wd = np.dot(r_oc, np.dot(s*mtx_inv, uv_im) - p_co)

    return np.round(point_wd, decimals = 3)


def excaliboard_reprojection_error(test_angles):
    # public
    
    # test for the extrinsic calibration error
    print('***start testing the reprojection error of the extrinsic parameter') 
    in_sample_mean_error = 0
    out_of_sample_mean_error = 0
    out_count = 0
    for angle in test_angles:
        # calculate the reprojection error of the other worldframe points
        print('Place excaliboard at {} degree orientation and press "q" '.format(angle))
        time.sleep(2)
        objpoints, imgpoints = excaliboard_worldframe_points('', angle)
        imgpoints = excaliboard_center_correction(imgpoints, objpoints)

        imgpoints_wide = np.reshape(imgpoints, (-1,2)).T
        objpoints2_wide = imageframe_to_worldframe(imgpoints_wide)
        objpoints2 = np.reshape(objpoints2_wide.T, (-1,1,3))
        print('expected coordinates | projected coordinates \n', np.hstack((objpoints[:,0,:-1],objpoints2[:,0,:-1])))

        mean_error = cv.norm(objpoints[:,0,:-1], objpoints2[:,0,:-1], cv.NORM_L2)/len(objpoints)
        
        print('total mean error in (mm): {}'.format(mean_error))

        if angle == 0:
            in_sample_mean_error = mean_error
        else:
            out_of_sample_mean_error += mean_error
            out_count += 1

    if in_sample_mean_error != 0:
        print('***************************** \n', 'total in sample error in (mm): ', in_sample_mean_error)
    if out_of_sample_mean_error != 0:
        print('***************************** \n', 'total out of sample error in (mm): ', out_of_sample_mean_error/out_count)


def leave_one_out_reprojection_error():
    # public

    # no board rotation needed, iterating leave-one-out sampling-testing method on all points on the excaliboard
    mtx, dist = intrinsic_cam_para()
    angle = 0
    print('Place excaliboard at {} degree orientation and press "q" '.format(angle))
    time.sleep(2)
    objpoints, imgpoints = excaliboard_worldframe_points('', angle)
    imgpoints = excaliboard_center_correction(imgpoints, objpoints)
    objpoints2 = np.empty((0,1,3), float)

    for out_id in range(len(objpoints)):
        condition = np.full((len(objpoints),1), True)
        condition[out_id] = False
        retval, r_co_rod, p_co = cv.solvePnP(objpoints[condition], imgpoints[condition], mtx, dist)
        r_co, jacobian = cv.Rodrigues(r_co_rod)
        
        # img frame to world frame
        point_im = np.reshape(imgpoints[out_id], (-1, 2)).T
        r_oc = np.linalg.inv(r_co)
        mtx_inv = np.linalg.inv(mtx)
        p_co_im = np.dot(mtx, p_co)
        s = p_co_im[2,0]
        _, npoints = np.shape(point_im)
        uv_im = np.vstack((point_im, np.ones((1,npoints))))
        point_wd = np.dot(r_oc, np.dot(s*mtx_inv, uv_im) - p_co)
        point_wd = np.reshape(np.round(point_wd, decimals = 3),(-1,1,3))
        
        objpoints2 = np.append(objpoints2, point_wd, axis = 0)
    print(' expected array | leave-one-out iterated coordinates \n', np.hstack((objpoints[:,0,:-1], objpoints2[:,0,:-1])))
    mean_error = cv.norm(objpoints[:,0,:-1], objpoints2[:,0,:-1], cv.NORM_L2)/len(objpoints)
    print('***************************** \n', 'total reprojection error in (mm): ', mean_error)
          
    


if __name__ == "__main__":
    np.set_printoptions(suppress=True)
    # current_path = os.path.abspath(__file__)
    # data_path = os.path.join(current_path,'calibrationData','camera_calibration')
    # calib_path = os.path.join(current_path,'calibrationData')

    # # obtain calibration images
    # # objpoints, imgpoints, frame = fetching_from_camera(data_path)
    # objpoints, imgpoints, frame = fetching_from_file(data_path)
    # gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    
    # # obtain camera matrix, distorion factor
    # ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    # print('camera matrix: \n', mtx, '\ndistortion: \n', dist)

    # # save the intrinsic calibrated parameters
    # fname = "camera_calib_intrinsic.npy"
    # mtx_path = os.path.join(os.getcwd(), 'calibrationData/')
    # with open(mtx_path+fname, 'wb') as file:
    #     np.save(file, mtx)
    #     np.save(file, dist)
    #     print('camera intrinsic parameters saved')


    # # test for the re-projection error
    # mean_error = 0
    # for i in range(len(objpoints)):
    #     imgpoints2, _ = cv.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    #     error = cv.norm(imgpoints[i], imgpoints2, cv.NORM_L2)/len(imgpoints2)
    #     mean_error += error
    # print( "total error in pixels: {}".format(mean_error/len(objpoints)) )

    # access saved intrinsic parameters
    mtx, dist = intrinsic_cam_para()
    print('camera matrix: \n', mtx, '\ndistortion: \n', dist)
 
    # calibrate camera extrinsic parameters
    # extrinsic_confirmation(calib_path)
    #excaliboard_reprojection_error([0, 60, 120, 180])

    # leave_one_out_reprojection_error()
           
            
            
            
