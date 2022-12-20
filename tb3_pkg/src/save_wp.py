import cv2
import yaml
from yaml.loader import SafeLoader

file_wp = open("wp.txt","w") # write mode
file_wp.write("")
file_wp.close()

def map_yaml_parse():
    with open('map.yaml', 'r') as f:
        data = yaml.load(f, Loader=SafeLoader)
        resolution = data['resolution']
        origin_x = data['origin'][0]
        origin_y = data['origin'][1]
        origin_phi = data['origin'][2]
        negate = data['negate']
        occupied_thresh = data['occupied_thresh']
        free_thresh = data['free_thresh']

        return origin_x, origin_y, resolution

def map_analyse(x_,y_):
    # get dimensions of image
    dimensions = img.shape

    # height, width, number of channels in image
    height_px = dimensions[0]
    width_px = dimensions[1]
    channels = dimensions[2]

    origin_x, origin_y, resolution = map_yaml_parse()

    map_height = height_px*resolution
    map_width  = width_px *resolution

    wp_x = origin_x + resolution*x_
    wp_y = origin_y + resolution*(height_px - y_)

    return wp_x, wp_y


def click_event(event, x, y, flags, params):
    # checking for left mouse clicks
    if event == cv2.EVENT_LBUTTONDOWN:
        # print(x, ' ', y) # printing the coordinates
        wp_x, wp_y = map_analyse(x,y)
        print(wp_x, wp_y)
        file_wp = open("wp.txt","a") # append mode
        file_wp.write(str(wp_x) + ", " + str(wp_y) + "\n")
        file_wp.close()

        # displaying the coordinates
        font = cv2.FONT_HERSHEY_SIMPLEX
        # cv2.putText(img, str(round(wp_x,2)) + ',' + str(round(wp_y,2)), (x,y), font, 1, (255, 0, 0), 2)

        color = (255, 0, 0)
        markerType = cv2.MARKER_CROSS
        markerSize = 15
        thickness = 2
        cv2.drawMarker(img, (x, y), color, markerType, markerSize, thickness)

        cv2.imshow('image', img)

    # checking for right mouse clicks   
    if event==cv2.EVENT_RBUTTONDOWN:
        # print(x, ' ', y) # printing the coordinates
        wp_x, wp_y = map_analyse(x,y)
        print(wp_x, wp_y)
        file_wp = open("wp.txt","a") # append mode
        file_wp.write(str(wp_x) + ", " + str(wp_y) + "\n")
        file_wp.close()

        # displaying the coordinates
        font = cv2.FONT_HERSHEY_SIMPLEX 
        b = img[y, x, 0]
        g = img[y, x, 1]
        r = img[y, x, 2]
        # cv2.putText(img, str(b) + ',' + str(g) + ',' + str(r), (x,y), font, 1, (255, 255, 0), 2)

        color = (0, 0, 255)
        markerType = cv2.MARKER_CROSS
        markerSize = 15
        thickness = 2
        cv2.drawMarker(img, (x, y), color, markerType, markerSize, thickness)

        cv2.imshow('image', img)


if __name__=="__main__":

    img = cv2.imread('map.pgm', 1) # read image
    
    cv2.imshow('image', img) #display image

    cv2.setMouseCallback('image', click_event) # setting mouse handler for the image and calling the click_event() function

    cv2.waitKey(0) # wait for a key to be pressed to exit

    cv2.destroyAllWindows() # close the window
