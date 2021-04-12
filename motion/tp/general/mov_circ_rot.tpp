#***************** ;
#circular Move ;
#--------------- ;
#for moving robot linearly to ;
#next pose in karel path ;
#***************** ;

TP_GROUPMASK = "1,1,*,*,*"
TP_COMMENT = "arc move"

case poseCode
  when PTH_MOVETO
    linear_move.to(pr1).at(speed, 'mm/s').term(FINE).coord
  when PTH_CLOSE
    circular_move.mid(pr1).to(pr2).at(speed, 'mm/s').term(termn).coord
    linear_move.to(pr2).at(speed, 'mm/s').term(FINE).coord
  else
    circular_move.mid(pr1).to(pr2).at(speed, 'mm/s').term(termn).coord
end
