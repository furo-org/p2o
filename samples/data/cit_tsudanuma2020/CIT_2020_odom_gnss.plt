set view equal xyz
#set size ratio -1
set terminal png
set output 'CIT_2020_odom_gnss.p2o_in.png'
splot 'CIT_2020_odom_gnss.p2o_in.txt' every::2 using 1:2:3 w l title 'before optimization'

set output 'CIT_2020_odom_gnss.p2o_out.png'
splot 'CIT_2020_odom_gnss.p2o_out.txt' every::2 using 1:2:3 w l title 'after optimization'
