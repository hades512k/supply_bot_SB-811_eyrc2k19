import numpy as np
import cv2
import serial
import math
import cv2.aruco as aruco
import csv

cap = cv2.VideoCapture(0)
coins_dispatched = 0
ser = 0

coins_dispatched = 0

def warp_image(image):

    grayscale = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    retval,thresh = cv2.threshold( grayscale,0,255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    thresh = cv2.bitwise_not(thresh)

    contours,hierarchy = cv2.findContours(thresh,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_TC89_L1)

    for contour in contours :
        if cv2.contourArea(contour) > 10000 :
            x,y,width,height = cv2.boundingRect(contour)

            pts1 = np.float32([[x,y],[x+width,y],[x,y+height],[x+width,y+height]])
            pts2 = np.float32([[0,0],[width,0],[0,width],[width,width]])
            transformation_matrix = cv2.getPerspectiveTransform(pts1,pts2)

            warped_image = cv2.warpPerspective(image, transformation_matrix,(width,width))

            return warped_image,width


def detect_Aruco(img):
    aruco_list = {}
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_50)
    parameters = aruco.DetectorParameters_create()  #refer opencv page for clarification
    #lists of ids and the corners beloning to each id
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters = parameters)
    #corners is the list of corners(numpy array) of the detected markers. For each marker, its four corners are returned in their original order (which is clockwise starting with top left). So, the first corner is the top left corner, followed by the top right, bottom right and bottom left.
    # print len(corners), corners, ids
    img = aruco.drawDetectedMarkers(img, corners,ids)

    if len(corners):    #returns no of arucos
        for k in range(len(corners)):
            temp_1 = corners[k]
            temp_1 = temp_1[0]
            temp_2 = ids[k]
            temp_2 = temp_2[0]
            aruco_list[temp_2] = temp_1
        image1 = mark_Aruco(img,aruco_list)
        cv2.imshow('image processing',image1)
        cv2.moveWindow('image processing',10,10)
        return aruco_list


def mark_Aruco(img, aruco_list):
    key_list = aruco_list.keys()
    font = cv2.FONT_HERSHEY_SIMPLEX
    for key in key_list:
        dict_entry = aruco_list[key]    #dict_entry is a numpy array with shape (4,2)
        centre = dict_entry[0] + dict_entry[1] + dict_entry[2] + dict_entry[3]#so being numpy array, addition is not list addition
        centre[:] = [int(x / 4) for x in centre]    #finding the centre
        orient_centre = centre + [0.0,5.0]
        centre = tuple(centre)
        orient_centre = tuple((dict_entry[0]+dict_entry[1])/2)
        #print centre
        #print orient_centre
        cv2.circle(img,centre,1,(0,0,255),8)
        cv2.circle(img,tuple(dict_entry[0]),1,(0,0,255),8)
        cv2.circle(img,tuple(dict_entry[1]),1,(0,255,0),8)
        cv2.circle(img,tuple(dict_entry[2]),1,(255,0,0),8)
        cv2.circle(img,orient_centre,1,(0,255,255),8)
        cv2.line(img,centre,orient_centre,(255,0,0),4) #marking the centre of aruco
        image1 = cv2.putText(img, str(key), (int(centre[0] + 20), int(centre[1])), font, 1, (0,0,255), 2, cv2.LINE_AA) # displaying the idno
    return image1



def detect_Aruco_centre(aruco_list):
    key_list = aruco_list.keys()
    for key in key_list:
        dict_entry = aruco_list[key]    #dict_entry is a numpy array with shape (4,2)
        centre = dict_entry[0] + dict_entry[1] + dict_entry[2] + dict_entry[3]#so being numpy array, addition is not list addition
        centre[:] = [int(x / 4) for x in centre]    #finding the centre
        #print("aruco centre",centre)
    return centre


def detect_arena_centre(input_image):
    res = cv2.inRange(input_image, np.array([150, 150 , 150]), np.array([ 255, 255, 255]))

    rows = input_image.shape[0]
    columns = input_image.shape[1]

    mask = np.zeros_like(res)
    cv2.circle(mask,(rows//2,columns//2),columns//16,(255,255,255),-1)
    res = cv2.bitwise_and(mask,res)
    #cv2.imshow('w',res)

    blur = cv2.GaussianBlur(res,(5,5),0)
    edges = cv2.Canny(blur,100,200)
    #cv2.imshow('e',edges)
    contours, hierarchy = cv2.findContours(edges,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_TC89_KCOS)

    #cv2.drawContours(input_image,contours,-1,(0,255,0),3)
    #cv2.imshow('c',input_image)
    M = cv2.moments(contours[0])

    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])

    arena_centre = (cx,cy)

    return arena_centre

def detect_red_coin(ip_image1,arena_centre,dia):
    ip_image = cv2.cvtColor(ip_image1,cv2.COLOR_BGR2HSV)
    #detecting the red colored regions of the arena
    red = cv2.inRange(ip_image, np.array([170,150,100]),np.array([180,255,255]))
    #cv2.imshow('red res',red)
    #bg will be the image on which mask will be drawn
    mask = np.zeros_like(red)
    mask = cv2.circle(mask,arena_centre,int(0.55 * dia // 2),(255,0,0),20)
    #cv2.imshow('m',mask)
    red = cv2.bitwise_and(mask,red)
    #cv2.imshow('rr',red)
    blur = cv2.GaussianBlur(red,(5,5),0)
    edges = cv2.Canny(blur,100,200)
    #cv2.imshow('e',edges)

    contours, hierarchy = cv2.findContours(edges,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_TC89_KCOS)
    print("cont length",len(contours))

    #cv2.drawContours(ip_image1,contours,-1,(0,255,0),3)
    #cv2.imshow('c',ip_image1)

    for cnt in contours :
            M = cv2.moments(cnt)
            rx = int(M['m10']/M['m00'])
            ry = int(M['m01']/M['m00'])
    cv2.waitKey(0)

    red_coin_coord = (rx,ry)

    return red_coin_coord

def detect_green_coin(frame,arena_centre,dia):

    hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)

    res = cv2.inRange(hsv, np.array([20,50,100]),np.array([70,255,255]))
    #cv2.imshow('before',res)
    kernel = np.ones((5,5),np.uint8)
    res = cv2.erode(res,kernel,iterations = 1)
    #cv2.imshow('res',res)
    mask = np.zeros_like(res)

    mask = cv2.circle(mask,arena_centre,int(0.55 * dia // 2),(255,0,0),10)
    #cv2.imshow('mask',mask)
    res = cv2.bitwise_and(mask,res)
    res = cv2.dilate(res, kernel,iterations = 1)
    #cv2.imshow('g',res)

    blur = cv2.GaussianBlur(res,(5,5),0)
    edges = cv2.Canny(blur,100,200)
    #cv2.imshow('e',edges)

    contours, hierarchy = cv2.findContours(edges,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_TC89_KCOS)
    print("cont length",len(contours))

    #cv2.drawContours(frame,contours,-1,(0,255,0),3)
    #cv2.imshow('c',frame)

    g_coin = list()

    for cnt in contours :
            M = cv2.moments(cnt)
            g_x = ( int(M['m10']/M['m00']))
            g_y = ( int(M['m01']/M['m00']))
            g_coin.append([g_x,g_y])

    cv2.waitKey(0)
    return g_coin

def find_angle(x1,y1,x2,y2,cx,cy) :
    angle = round((math.degrees(math.atan2(y2-cy,x2-cx)-math.atan2(y1-cy,x1-cx))),4)
    return int(angle) if angle > 0 else  360 + angle

def find_node_number(img,arena_centre,diameter,aruco_centre,red_angle,green_angle):

    res = cv2.inRange(img,np.array([120,120,120]),np.array([255,255,255]))
    #cv2.imshow('k',res)

    mask = np.zeros_like(res)
    cv2.circle(mask,arena_centre,int(0.8 * diameter // 2),(255,255,255),20)
    mask = cv2.circle(mask,aruco_centre,int( 0.15 * diameter // 2),(0,0,0),-1)
    res = cv2.bitwise_and(res,mask)
    #cv2.imshow('l',res)

    kernel = np.ones((2,2),np.uint8)
    erosion = cv2.erode(res,kernel,iterations = 5 )
    #cv2.imshow('2',erosion)

    dilate = cv2.dilate(erosion,kernel,iterations = 5)
    #cv2.imshow('3',dilate)

    blur = cv2.GaussianBlur(dilate,(5,5),0)
    edges = cv2.Canny(blur,100,200)
    #cv2.imshow('e',edges)

    contours, hierarchy = cv2.findContours(edges,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_TC89_KCOS)
    print("cont length",len(contours))

    #cv2.drawContours(img,contours,-1,(0,255,0),3)
    #cv2.imshow('c',img)

    nodes = list()


    if len(contours) == 8 :
        for cnt in contours :
            M = cv2.moments(cnt)
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            nodes.append([cx,cy])

    ang = list()

    for i in nodes :
        angle = find_angle(i[0],i[1],aruco_centre[0],aruco_centre[1],arena_centre[0],arena_centre[1])
        #print("node angle:",angle)
        ang.append(angle)
        #print("appended")

    ang.sort(reverse = True)
    #print(ang)
    #ang = ang.sort(reverse = True)

    #print("nodes",nodes)
    green_coin_node = list()
    red_coin_node = list()

    for j in ang :
        if (j <= (red_angle + 7)) and (j >= (red_angle - 7)):
            print("red coin is identified at:", ang.index(j)+2)
            red_coin_node.append(ang.index(j)+2)
            continue
        for i in range(len(green_angle)):
            if (j <= (green_angle[i] + 7)) and (j >= (green_angle[i] - 7)):

                print("green coin "+str(i)+" is identified at:", ang.index(j)+2)
                green_coin_node.append(ang.index(j)+2)
                continue

    with open('Run_SupplyBot.csv','w') as file :
        writer = csv.writer(file)
        writer.writerow(["Node No.","Type of relief aid"])
        for i in range(len(red_coin_node)):
            writer.writerow([str(red_coin_node[i]),"Medical Aid"])
        for i in range(len(green_coin_node)):
            writer.writerow([str(green_coin_node[i]),"Food Supply Aid"])

    file.close()


def run(destination,arena_centre):

    global coins_dispatched
    global ser

    while(True):

        ret,frame = cap.read()
        cv2.imshow('camera',frame)
        cv2.moveWindow('camera', 800,10)
        cv2.waitKey(1)
        warped_frame,w = warp_image(frame)

        aruco_list = detect_Aruco(warped_frame)
        try :
            if(len(aruco_list)):
                aruco_centre = detect_Aruco_centre(aruco_list)
                aruco_centre = (aruco_centre[0],aruco_centre[1])
        except:
            print("aruco not identified")
            continue

        angle = find_angle(destination[0],destination[1],aruco_centre[0],aruco_centre[1],arena_centre[0],arena_centre[1])

        flag = 0

        if( 4 < angle < 357):
            ser.write(b'F')
            print("sending F")
        else :
            if(coins_dispatched < 3):
                ser.write(b'H')
                print("sending H")
                while(True):
                    ret,frame = cap.read()
                    cv2.imshow('camera',frame)
                    cv2.waitKey(1)
                    #if any data is ready to be read in serial port
                    if(ser.inWaiting()>0):
                        #wait to recieve acknowledgement for dispatch from arduino
                        print("coin dispatched")
                        rcv = ser.read()
                        if(rcv == b'D'):
                            coins_dispatched = coins_dispatched + 1
                            print("coins dispatched",coins_dispatched)
                            break

                return True

            else:

                ser.write(b'S')
                print("sending S")
                for iy in range(200):
                    ret,frame = cap.read()
                    cv2.imshow('camera',frame)
                    cv2.waitKey(1)
                return True

def main():

    global cap
    global coins_dispatched
    global ser



    cap = cv2.VideoCapture(1)
    print("coins dispatched from main",coins_dispatched)
    try :
        ser = serial.Serial('COM7')
    except:
        print("Xbee not connected or Xbee is busy")
        return False

    ret,frame = cap.read()
    cv2.imshow('camera',frame)
    cv2.moveWindow('camera', 800,10)

    warped_frame,w = warp_image(frame)
    #cv2.imshow('warped',warped_frame)
    cv2.waitKey(0)

    arena_centre = detect_arena_centre(warped_frame)
    while(True):
        aruco_list = detect_Aruco(warped_frame)
        try:
            if(len(aruco_list)):
                aruco_centre = detect_Aruco_centre(aruco_list)
                capital_node_coords = aruco_centre.copy()
                aruco_centre = (aruco_centre[0],aruco_centre[1])
                break
        except:
            print("aruco not identified")
            continue
    red_coin_coord = detect_red_coin(warped_frame,arena_centre,w)

    green_coin_coord = detect_green_coin(warped_frame,arena_centre,w)
    print("green coin coords",green_coin_coord)
    red_coin_angle = find_angle(red_coin_coord[0],red_coin_coord[1],aruco_centre[0],aruco_centre[1],arena_centre[0],arena_centre[1])
    print("red angle",red_coin_angle)

    green_angle = list()

    for i in green_coin_coord :
        #ang1 = angle_calculate_2((i[0],i[1]),aruco_centre,centre)
        ang1 = find_angle(i[0],i[1],aruco_centre[0],aruco_centre[1],arena_centre[0],arena_centre[1])
        print("ang1e bw g n a",ang1)
        green_angle.append(ang1)

    find_node_number(warped_frame, arena_centre,w,aruco_centre,red_coin_angle,green_angle)

    run(red_coin_coord,arena_centre)
    while(True):
        if(0 < coins_dispatched < 3):
            print("entered here")
            while(True):
                ret,frame = cap.read()

                warped_frame,w = warp_image(frame)

                aruco_list = detect_Aruco(warped_frame)
                try:
                    if(len(aruco_list)):
                        aruco_centre = detect_Aruco_centre(aruco_list)
                except :
                    print("aruco not identified")
                    continue

                minimum = 360

                for i  in green_coin_coord :
                    #ang1 = angle_calculate_2((i[0],i[1]),aruco_centre,centre)
                    ang1 = find_angle(i[0],i[1],aruco_centre[0],aruco_centre[1],arena_centre[0],arena_centre[1])
                    print("angle value",ang1)
                    if(ang1 < minimum):
                        minimum = ang1
                        target_coord = (i[0],i[1])
                        final_index = green_coin_coord.index(i)

                del green_coin_coord[final_index]
                print("green coin coords",green_coin_coord)
                print("target",target_coord)
                run(target_coord,arena_centre)
                print("back to main, value of coin dispatched",coins_dispatched)
                break

        if(coins_dispatched == 3):
            print("last run")
            print("coins dispatched from last run",coins_dispatched)
            run(capital_node_coords,arena_centre)
            return True

if __name__ == '__main__' :
    main()



'''
        if(180 < angle < 355 ):
            ser.write(b'E')
            if(flag == 0):
                while(ser.inWaiting()<0):
                    pass
                if(ser.inWaiting()>0):
                    rcv = ser.read()
            if(rcv == b'G' or flag == 1):
                ser.write(b'B')
                flag = 1
        if( 5 < angle <= 180):
            ser.write(b'F')

        if( 355 < angle or angle < 5):
            ser.write(b'H')

            while(True):
                ret,frame = cap.read()
                cv2.imshow('camera',frame)
                cv2.waitKey(1)
                #if any data is ready to be read in serial port
                if(ser.inWaiting()>0):
                    #wait to recieve acknowledgement for dispatch from arduino
                    print("coin dispatched")
                    rcv = ser.read()
                    if(rcv == b'D'):
                        coins_dispatched = coins_dispatched + 1
                        break
'''
