# This script draws trajectory from the last raspilot flight

stats 'trajectory.dat' using 1:2 every :::0::0  name 'A'
dx(x) = (xd = (px==NaN?0:x-px), px = x, xd)
dy(y) = (yd = (py==NaN?0:y-py), py = y, yd)
set xrange [A_min_x-0.01:A_max_x+0.01]
set yrange [A_min_y-0.01:A_max_y+0.01]
set pointsize 2
do for [ii=20:A_records] {
  px=NaN
  py=NaN
  plot 'trajectory.dat' every ::(ii-20)::(ii-10) using (px):(py):(dx($1)):(dy($2)) w vector lc rgb "gray" notitle, 'trajectory.dat' every ::(ii-10)::(ii-5) using (px):(py):(dx($1)):(dy($2)) w vector lc rgb "black"  notitle, 'trajectory.dat' every ::(ii-5)::(ii-1) using (px):(py):(dx($1)):(dy($2)) w vector lc rgb "blue"  notitle, 'trajectory.dat' every ::(ii-1)::(ii) using (px):(py):(dx($1)):(dy($2)) w vector lc rgb "red"  notitle, '' u 1:2 every ::ii::ii w p pt 7 lc rgb "red" notitle
  pause 0.1
}
pause 5
