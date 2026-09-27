# SPDX-License-Identifier: MIT
# fpga/tcl/build.tcl — Deimos Vivado batch build driver
#
# Extends styx build procs with deimos-specific fingerprinting
# (includes both styx RTL + deimos RTL) and deimos synth knobs.
#
# Usage:
#   vivado -mode batch -nojournal -nolog -source fpga/tcl/build.tcl -tclargs \
#       -project_dir fpga/project \
#       -rtl_dir fpga/rtl \
#       -adi_hdl_dir platform/styx/fpga/extern/adi-hdl \
#       -output_dir build/fpga \
#       [-place_strategy Explore] \
#       [-opt_directive ExploreArea] \
#       [-ooc_modules "mod1 mod2 ..."] \
#       [-incremental path/to/reference.dcp] \
#       [-skip_project]

set styx_tcl_dir [file normalize [file dirname [info script]]/../../platform/styx/fpga/tcl]
source [file join $styx_tcl_dir styx_procs.tcl]

# ============================================================================
# Argument parsing
# ============================================================================

set opts [styx::parse_args $argv]

set project_dir    [file normalize [dict get $opts project_dir]]
set rtl_dir        [file normalize [dict get $opts rtl_dir]]
set adi_hdl_dir    [file normalize [dict get $opts adi_hdl_dir]]
set output_dir     [file normalize [dict get $opts output_dir]]
set place_strategy [dict get $opts place_strategy]
set opt_directive  [dict get $opts opt_directive]
set ooc_modules    [dict get $opts ooc_modules]
set incremental    [dict get $opts incremental]

# Styx RTL directory (included in combined fingerprint)
set styx_rtl_dir [file normalize [file dirname [info script]]/../../platform/styx/fpga/rtl]

file mkdir $output_dir

# ============================================================================
# Phase 1.5: Build fingerprint (computed BEFORE project creation)
#
# Computed here because Phase 1 (create_project) sources system_project.tcl
# and both system_bd.tcl files at global scope; styx system_bd.tcl sets the
# global `rtl_dir` to the styx RTL dir, clobbering this script's rtl_dir.
# If the fingerprint ran after Phase 1, it would hash the styx RTL twice and
# no deimos RTL at all, making the fingerprint invariant to every deimos RTL
# change. Capture the paths in uniquely named variables and hash the
# untouched source tree instead.
# ============================================================================

set deimos_fp_styx_rtl_dir $styx_rtl_dir
set deimos_fp_rtl_dir $rtl_dir
set fingerprint [styx::fingerprint [list $deimos_fp_styx_rtl_dir $deimos_fp_rtl_dir] $project_dir]

# ============================================================================
# Phase 1: Project creation
# ============================================================================

if {![dict get $opts skip_project]} {
    puts "========== PHASE 1: PROJECT CREATION =========="
    styx::create_project $adi_hdl_dir $project_dir
} else {
    puts "========== PHASE 1: OPENING EXISTING PROJECT =========="
    open_project $project_dir/pluto.xpr
    set bd_file [glob -nocomplain $project_dir/pluto.srcs/sources_1/bd/system/system.bd]
    if {[llength $bd_file] > 0} {
        open_bd_design [lindex $bd_file 0]
        puts "Opened existing block design"
    }
}

# ============================================================================
# Phase 2: Apply build fingerprint (computed pre-Phase-1)
# ============================================================================

puts "========== PHASE 2: BUILD FINGERPRINT =========="
puts "Fingerprint: $fingerprint"
styx::set_build_id $fingerprint -project_id 0x57494649

# ============================================================================
# Phase 3: OOC synthesis (deimos: AreaOptimized_high, cset_opt=4)
# ============================================================================

if {[llength $ooc_modules] > 0} {
    puts "========== PHASE 3: OOC SYNTHESIS ([llength $ooc_modules] modules) =========="
    styx::ooc_synth {*}$ooc_modules -strategy Flow_AreaOptimized_high -control_set_opt 4
} else {
    puts "========== PHASE 3: OOC SYNTHESIS (skipped — no modules specified) =========="
}

# ============================================================================
# Phase 4: Global synthesis (deimos: full flatten)
# ============================================================================

puts "========== PHASE 4: GLOBAL SYNTHESIS =========="
styx::global_synth system_top Flow_AreaOptimized_high -flatten full

# ============================================================================
# Phase 5: Implementation
# ============================================================================

puts "========== PHASE 5: IMPLEMENTATION (strategy=$place_strategy opt=$opt_directive) =========="

if {$incremental ne "" && [file exists $incremental]} {
    puts "INCREMENTAL: Using reference checkpoint: $incremental"
    set_property INCREMENTAL_CHECKPOINT $incremental [get_runs impl_1]
}

if {$opt_directive ne ""} {
    styx::implement $place_strategy -opt_directive $opt_directive
} else {
    styx::implement $place_strategy
}

# ============================================================================
# Phase 6: Reports
# ============================================================================

puts "========== PHASE 6: TIMING + UTILIZATION =========="

set timing [styx::check_timing]
set util [styx::report_util $output_dir]

# ============================================================================
# Phase 7: Write outputs
# ============================================================================

puts "========== PHASE 7: WRITE OUTPUTS =========="
styx::write_outputs $output_dir

set dcp_path [file join $output_dir deimos_impl.dcp]
write_checkpoint -force $dcp_path
puts "DCP: $dcp_path (use with -incremental for faster rebuilds)"

set fp_fd [open [file join $output_dir fingerprint] w]
puts $fp_fd $fingerprint
close $fp_fd

# ============================================================================
# Phase 7.5: Build result identity
#
# The fingerprint identifies the SOURCE tree. A placement re-run of the
# same tree produces a different netlist with the same fingerprint, so
# result-level identity (bitstream hash + build metadata) is recorded
# separately and logged by the session scripts.
# ============================================================================

set bit_file [file join $output_dir system_top.bit]
if {[file exists $bit_file]} {
    if {[catch {exec sha256sum $bit_file} bsha_out]} {
        set bsha_out [exec shasum -a 256 $bit_file]
    }
    set bsha [lindex [split [string trim $bsha_out]] 0]
    set bsha_fd [open [file join $output_dir bitstream.sha256] w]
    puts $bsha_fd $bsha
    close $bsha_fd
    puts "BITSTREAM SHA256: $bsha"
} else {
    set bsha "unavailable"
    puts "BITSTREAM SHA256: unavailable (no system_top.bit)"
}

set build_info [dict create \
    fingerprint $fingerprint \
    bitstream_sha256 $bsha \
    wns [dict get $timing wns] \
    whs [dict get $timing whs] \
    luts [dict get $util luts] \
    ffs [dict get $util ffs] \
    bram [dict get $util bram] \
    dsp [dict get $util dsp] \
    place_strategy $place_strategy \
    opt_directive $opt_directive \
    incremental [expr {$incremental ne "" ? $incremental : "none"}] \
    host [expr {[info exists ::env(DEIMOS_BUILD_HOST)] ? $::env(DEIMOS_BUILD_HOST) : "unspecified"}] \
    timestamp [clock format [clock seconds] -format %Y-%m-%dT%H:%M:%SZ -gmt 1]]

set info_fd [open [file join $output_dir build_info.json] w]
puts $info_fd "{"
set first 1
dict for {k v} $build_info {
    if {!$first} { puts $info_fd "," }
    set first 0
    puts $info_fd "  \"$k\": \"$v\""
}
puts $info_fd "}"
close $info_fd
puts "BUILD INFO: [file join $output_dir build_info.json]"

# ============================================================================
# Done
# ============================================================================

if {[dict get $timing met]} {
    puts "\n========== BUILD SUCCESS =========="
    puts "  Fingerprint: $fingerprint"
    puts "  Timing:      MET (WNS=[dict get $timing wns]ns)"
    puts "  Output:      $output_dir"
    exit 0
} else {
    puts "\n========== BUILD COMPLETE (TIMING VIOLATED) =========="
    puts "  Fingerprint: $fingerprint"
    puts "  WNS:         [dict get $timing wns]ns"
    puts "  Strategy:    $place_strategy"
    puts "  Output:      $output_dir (bitstream generated despite violation)"
    puts ""
    puts "  Try: make bitstream PLACE_STRATEGY=EarlyBlockPlacement"
    puts "  Try: make bitstream PLACE_STRATEGY=AltSpreadLogic_high"
    exit 2
}
