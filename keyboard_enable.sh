# This script remaps some keys for Shooterz game. d means disable, e means enable
if [[ ${1} == "e" ]]
then 
  xmodmap -e 'keycode 67 = 3'#F1
  xmodmap -e 'keycode 10 = 7 7' #1
  xmodmap -e 'keycode 11 = 8 8'#2
  xmodmap -e 'keycode 14 = 4 4'#5
  xmodmap -e 'keycode 15 = 7 7'#6
  xmodmap -e 'keycode 17 = 7 7'#8
else
  xmodmap -e 'keycode 67 = F1'
  xmodmap -e "keycode 10 = 1 0x0021"#1
  xmodmap -e 'keycode 11 = 2 0x0040'#2
  xmodmap -e 'keycode 14 = 5 0x0025'#5
  xmodmap -e 'keycode 15 = 6 0x005e'#6
  xmodmap -e 'keycode 17 = 8 0x002a'#8
fi

#xmodmap -pke - keycodes
#xmodm
