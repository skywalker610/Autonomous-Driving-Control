# import gmplot package 

import gmplot 

with open('/home/qiyang-isuzu/Desktop/ISUZU/ros_ws_DBW_CAN/lat_raw.txt') as f_lat:
    latitude_list = f_lat.read().splitlines()

latitude_list = list(map(float,latitude_list))

with open('/home/qiyang-isuzu/Desktop/ISUZU/ros_ws_DBW_CAN/lon_raw.txt') as f_lon:
    longitude_list = f_lon.read().splitlines()

longitude_list = list(map(float,longitude_list))
  
gmap3 = gmplot.GoogleMapPlotter(42.3842014,-83.5005484, 14, apikey='AIzaSyAXFZ4CBWewIGa1-NEiZ9gbM_uSal4Ot9M') #apikey='AIzaSyBIvup4Q9gQGeOMvYnH-XdXadiAd-1KeYE') 
# scatter method of map object  
# scatter points on the google map 
#gmap3.scatter( latitude_list, longitude_list, '#FF0000', size = 1, marker = False ) 
  
# Plot method Draw a line in 
# between given coordinates 
gmap3.plot(latitude_list, longitude_list,'cornflowerblue', edge_width = 2) 

gmap3.draw( "my_gm_plot.html" )

