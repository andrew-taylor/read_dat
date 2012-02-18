#!/bin/sh
# Usage read_dat3.sh [tape_drive] [read_dat_options]
# e.g read_dat3.sh /dev/st0 -p tape15_
# read a audio DAT tape times
# then correct erors between images
# then runs read dat to extract track
#	Andrew Taylor (andrewt@cse.unsw.edu.au)
PATH=.:$PATH
TMP=raw_tape_image.$$
TAPE_DRIVE=$1
shift

mt -f "$TAPE_DRIVE" status
echo Reading tape into $TMP.1
dd 'bs=5822' if="$TAPE_DRIVE" of="$TMP.1"
echo Tape read 
ls -l "$TMP.1"
echo Rewinding tape
mt -f "$TAPE_DRIVE" rewind
mt -f "$TAPE_DRIVE" status
echo Reading tape into $TMP.2
dd 'bs=5822' if="$TAPE_DRIVE" of="$TMP.2"
#echo adding errors for testing;add_error $TMP.2 >$TMP.2.errors && mv $TMP.2.errors $TMP.2
echo Tape read 
ls -l "$TMP.2"
echo Rewinding tape
mt -f "$TAPE_DRIVE" rewind
mt -f "$TAPE_DRIVE" status
echo Reading tape into $TMP.3
dd 'bs=5822' if="$TAPE_DRIVE" of="$TMP.3"
#echo adding errors for testing;add_error $TMP.3 >$TMP.3.errors && mv $TMP.3.errors $TMP.3
echo Tape read 
ls -l "$TMP.3"
echo Rewinding tape
mt -f "$TAPE_DRIVE" rewind
echo Combining $TMP.1 $TMP.2 $TMP.3 into $TMP
triple_merge "$TMP.1" "$TMP.2" "$TMP.3" >"$TMP"
echo Removing "$TMP.1" "$TMP.2" "$TMP.3 "
rm -f "$TMP.1" "$TMP.2" "$TMP.3" 
echo Running read_dat $TMP "$@"
read_dat "$TMP" "$@"
