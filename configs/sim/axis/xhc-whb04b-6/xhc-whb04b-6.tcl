#-----------------------------------------------------------------------
# Copyright: 2018
# Author:    Raoul Rubien <https://github.com/rubienr>
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
#-----------------------------------------------------------------------

# xhc-whb04b-6.tcl: HALFILE for xhc-whb04-6 pendant
#
# Usage:
# In ini file:
#
#   [XHC_WHB04B_6_CONFIG]
#   # optional, yes|no, default yes
#   PENDANT_REQUIRED = yes 
#
#   # optional, see xhc-whb04b.tcl --help, i.e. all log facilities "-a" 
#   COMPONENT_ARGUMENTS = "-a"
#   ...
#   [HAL]
#   HALFILE = xhc-whb04b-6.tcl
#   ...
#   HALFILE = <other_hal_file>

set ::component_name "xhc-whb04b-6"
set ::script_name "$::component_name.tcl"
set ::pendant_required "yes"
set ::component_arguments ""

puts stdout "$::script_name .ini variables:"
foreach name [array names ::XHC_WHB04B_6_CONFIG] {
    puts stdout "::XHC_WHB04B_6_CONFIG($name) = $::XHC_WHB04B_6_CONFIG($name)"
}

if [info exists ::XHC_WHB04B_6_CONFIG(PENDANT_REQUIRED)] {
    set ::pendant_required &::XHC_WHB04B_6_CONFIG(PENDANT_REQUIRED)
}

if [info exists ::XHC_WHB04B_6_CONFIG(COMPONENT_ARGUMENTS)] {
    set ::component_arguments $::XHC_WHB04B_6_CONFIG(COMPONENT_ARGUMENTS)
}

#-----------------------------------------------------------------------

proc popup_msg {msg} {
    puts stderr "$msg"
    if [catch 
    {
        package require Tk
        wm withdraw .
        tk_messageBox -title "$::script_name: loadusr" -type ok -message "$msg"
        destroy .
    } msg] {
        puts stderr "$msg"
    }
}; # popup_msg

#-----------------------------------------------------------------------

proc err_exit {msg} {
    puts stderr "\n$::script_name: $msg\n"
    exit 1
}; # err_exit

# ----------------------------------------------------------------------

set dashx -x
switch $::pendant_required {
    no {set dashx ""}
}

set cmd "loadusr -W xhc-whb04b-6 -H $::component_arguments"
puts stdout "loading component: $cmd"
if [catch {eval $cmd} msg] {
    set msg "\n"
    set msg "$msg $::script_name: <$cmd>\n"
    set msg "$msg Is it plugged in?\n"
    set msg "$msg Are permissions correct?\n"
    set msg "$msg Continuing without $component_name\n"
    set msg "$msg \nFailing cmd:\n$cmd\n"
    popup_msg "$msg"
    return; # not an exit
}
