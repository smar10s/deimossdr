# SPDX-License-Identifier: MIT
# fpga/project/system_project.tcl — Vivado project creation for Deimos
#
# Creates Vivado project with styx infrastructure + deimos receiver pipeline.
# Handles IP catalog setup and ADI IP registration before BD creation.
#
# Called by: build.tcl (once, or when BD/sources change)
# Expects: $ad_hdl_dir set before sourcing

# If ad_hdl_dir not set, derive from directory structure
if {![info exists ad_hdl_dir] || $ad_hdl_dir eq ""} {
    set ad_hdl_dir [file normalize [file dirname [info script]]/../../platform/styx/fpga/extern/adi-hdl]
}

set project_dir [file dirname [info script]]
set styx_rtl_dir [file normalize $project_dir/../../platform/styx/fpga/rtl]
set deimos_rtl_dir [file normalize $project_dir/../rtl]

# ---- Phase 1: Create project ----
set project_name pluto
set part "xc7z010clg225-1"

if {[file exists $project_dir/$project_name.xpr]} {
    puts "Project already exists, opening..."
    open_project $project_dir/$project_name.xpr
} else {
    create_project $project_name $project_dir -part $part
}

# ---- Phase 2: Source ADI helper procs and set globals ----
# Must be done before IP registration because IP scripts depend on them
source $ad_hdl_dir/projects/scripts/adi_project_xilinx.tcl
source $ad_hdl_dir/projects/scripts/adi_board.tcl
source $ad_hdl_dir/projects/common/xilinx/adi_fir_filter_bd.tcl
source $ad_hdl_dir/projects/scripts/adi_xilinx_msg.tcl

# ---- SmartConnect disabled on Zynq-7000 (resource + timing) ----
# adi_board.tcl defaults use_smartconnect=1; adi_project_create would set it
# to 0 for xc7z* devices. We create the project manually, so we must do it.
set use_smartconnect 0

# ---- Phase 3: Register ADI IPs in catalog ----
# The ADI library IPs are not auto-registered by update_ip_catalog in
# Vivado 2025.2. They must be registered by sourcing their *_ip.tcl scripts.
set ::env(ADI_IGNORE_VERSION_CHECK) 1
set IGNORE_VERSION_CHECK 1

set_property ip_repo_paths $ad_hdl_dir/library [current_project]
update_ip_catalog

foreach ip {axi_ad9361} {
    set ip_dir [file join $ad_hdl_dir/library $ip]
    set ip_tcl [file join $ip_dir ${ip}_ip.tcl]
    if {[file exists $ip_tcl]} {
        puts "Registering ADI IP: $ip"
        cd $ip_dir
        source $ip_tcl
        cd $project_dir
        # IP script uses ipx::current_core which may shift context; re-establish
        if {[get_projects -quiet] ne ""} {
            current_project $project_name
        }
    }
}

# ---- Phase 4: Add source files ----
add_files -norecurse -fileset sources_1 [list \
    "$project_dir/system_top.v" \
    "$ad_hdl_dir/library/common/ad_iobuf.v" \
]
add_files -fileset constrs_1 -norecurse "$project_dir/constraints/system_constr.xdc"
add_files -fileset constrs_1 -norecurse "$project_dir/constraints/cdc_timing.xdc"

# Styx RTL (infrastructure — also added by styx system_bd.tcl, duplicates OK)
foreach f [lsort [glob $styx_rtl_dir/*.v]] {
    add_files -norecurse -fileset sources_1 $f
}

# Deimos RTL (receiver pipeline)
foreach f [lsort [glob $deimos_rtl_dir/*.v]] {
    add_files -norecurse -fileset sources_1 $f
}

# Hex files (deinterleaver ROM, etc.)
set hex_files [glob -nocomplain $deimos_rtl_dir/*.hex]
if {[llength $hex_files] > 0} {
    add_files -norecurse -fileset sources_1 $hex_files
    set_property FILE_TYPE {Memory Initialization Files} \
        [get_files -of_objects [get_filesets sources_1] *.hex]
}

set_property generic {} [current_fileset]
set_param general.maxThreads [exec nproc]

# ---- Phase 5: Create Block Design ----
if {[get_bd_designs -quiet] ne ""} {
    close_bd_design [get_bd_designs]
}
# Remove existing BD if present (idempotent rebuild)
set existing_bd [get_files -quiet -filter "NAME =~ *system.bd"]
if {[llength $existing_bd] > 0} {
    puts "Removing existing block design 'system'..."
    remove_files -quiet $existing_bd
}
file delete -force {*}[glob -nocomplain [file join $project_dir *.srcs sources_1 bd system]]
create_bd_design "system"

# Source the composition BD (sources styx BD + adds receiver pipeline)
source $project_dir/system_bd.tcl

# ---- Phase 6: Finalize ----
save_bd_design
validate_bd_design

# Disable auto-generated PS7 constraints
set ps7_xdc [get_files -quiet *system_sys_ps7_0.xdc]
if {[llength $ps7_xdc] > 0} {
    set_property is_enabled false $ps7_xdc
}

set system_bd [get_files $project_dir/$project_name.srcs/sources_1/bd/system/system.bd]
generate_target {synthesis implementation} $system_bd
export_ip_user_files -of_objects $system_bd -no_script -sync -force -quiet
create_ip_run $system_bd
make_wrapper -files $system_bd -top
import_files -force -norecurse -fileset sources_1 $project_dir/$project_name.srcs/sources_1/bd/system/hdl/system_wrapper.v

puts "Project created: $project_dir/$project_name.xpr"
