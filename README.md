# CurrentRanger RGr Updates
For original docu, see master branch.

## List of Updates
- as I found out, in most small projects, where many functions are called just once or twice, the -O2 performance optimization generates smaller code than the -Os optimization. Therefore I enabled -O2 in platform.txt for the C++ compiler.
- some code shrinking, using loops instead of repeating equal function calls
- used some C++ templates for increased readability
- decreased OLED display update speed from 5.5fps down to 5fps
- decreased touch sensor polling from 20sps down to 10sps
- refactored: removed some global variables by moving them into the contextual functions
- implemented: new serial option 'o' to toggle OLED display. That's because an OLED update requires about 17 ms and during this time no ADC sampling takes place
- implemented: if current ranger is connected to USB, increased the auto delay by a factor of 10x

*Side effect:* Reduced binary size by about 2%.
