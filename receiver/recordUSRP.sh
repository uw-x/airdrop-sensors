#!/bin/bash
# ./recordUSRP.sh -f 2402000000 -o ch37_pcb_A1_flipped.dat

HOST=root@192.168.1.102

while getopts o:f:d:s: option
do
case "${option}"
in
o) OUTPUT_FILE=${OPTARG};;
f) FREQUENCY=${OPTARG};;
d) DURATION=${OPTARG};;
s) SAMPLING_RATE=${OPTARG};;
esac
done

[ -z "$FREQUENCY" ] && echo "-f argument empty, pass in frequency (-f 2402000000)" && exit
[ -z "$OUTPUT_FILE" ] && echo "-o argument empty, pass in output file name (-o filename.dat)" && exit
[ -z "$DURATION" ] && echo "-d argument empty, pass in duration in seconds (-d 1)" && exit
[ -z "$SAMPLING_RATE" ] && echo "-s argument empty, pass in sampling rate in hz (-s 8000000)" && exit

NSAMPLES=$(($DURATION * $SAMPLING_RATE))
sshCommand="uhd_rx_cfile --antenna TX/RX --gain=20 --samp-rate="$SAMPLING_RATE" --nsamples="$NSAMPLES" --freq="$FREQUENCY
# sshCommand="uhd_rx_cfile --antenna TX/RX --gain=80 --samp-rate="$SAMPLING_RATE" --nsamples="$NSAMPLES" --freq="$FREQUENCY
# sshCommand="uhd_rx_cfile --antenna TX/RX --gain=40 --samp-rate="$SAMPLING_RATE" --nsamples="$NSAMPLES" --freq="$FREQUENCY
# sshCommand="uhd_rx_cfile --antenna TX/RX --samp-rate="$SAMPLING_RATE" --nsamples="$NSAMPLES" --freq="$FREQUENCY
sshCommand="${sshCommand} "$OUTPUT_FILE
rmCommand="rm -rf *.dat"

# delete file before running this command

ssh $HOST $rmCommand
ssh $HOST $sshCommand
scp $HOST:/home/root/$OUTPUT_FILE .

