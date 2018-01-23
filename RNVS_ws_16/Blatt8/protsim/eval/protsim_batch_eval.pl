#!/usr/bin/perl -w
no warnings "once";

#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
# Packages
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

use File::Basename;
use Getopt::Long;

#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
# Configurable Options
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

$DEBUG=0;

#$module = "ARQNetwork.LeftRouter.interfaceQueue[0]";
$module = "RoutingNetwork.Node2.interfaceQueue[1]";
$vector = "waiting time";


### Forward Declarations ###

sub readCmdLine();
sub openFile($);
sub closeFile();
sub readScalarLine();
sub mainLoop();


### MAIN ###

#scalar "RoutingNetwork.Node1.interfaceQueue[0]"         "waiting time.Batch-1"  0.0370483536

my $scafile = readCmdLine(); # sets @runs, $infilebase and $outfile
openFile($scafile);
mainLoop();
closeFile();

exit(0);


### Sub routines ###

sub readCmdLine() {
    my $help = 0;

    Getopt::Long::Configure ("bundling");
    GetOptions('help|?'=>\$help ) or $help = 2;

    if ($help) {
	print <<EOF;
protsim_batch_eval.pl scafile
protsim_batch_eval.pl -? | --help

EOF
	exit 1 if $help == 2;
	exit 0;
    }

    my $scafile = shift @ARGV or die "Input file name expected";

    return $scafile;
}


{ # enclosing the sub routine in a block, emulates static locals
    my $numval = -1;
    local *INPUT;

    sub openFile($) {
	my $name = shift or die;
	open(INPUT,"$name") or die "Could not open $name: $!";
    }

    sub closeFile() {
	close(INPUT);
    }

    sub readScalarLine() {
	my ($mod, $vec, $val, $batch);

	while (<INPUT>) {
	    if (($mod, $vec, $val) = /^\s*scalar\s+"([^"]*)"\s+"([^"]*)"\s+([0-9eE.+\-]+)\s+$/) {
		last if $mod eq $module && (($batch) = $vec =~ /^${vector}\.Batch(\d+)$/);
	    }
	}

	return undef if eof(INPUT);

	print "DEBUG: $batch: $val\n" if $DEBUG;

	die "Unexpected batch number" if $batch != $numval+1;
	++$numval;

	return $val;
    }
}




sub mainLoop() {
    my ($val, $num,$sum,$sqsum);

    $num = $sum = $sqsum = 0;

    while (defined($val = readScalarLine())) {
	$num++;
	$sum += $val;
	$sqsum += $val*$val;
    }

    my $mean = $sum/$num;
    print "Mean of batch means: $mean\n";
    print "Sample variance of means: ";
    my $var = -1;
    if ($num >= 2) {
	$var = ($sqsum-$sum*$sum/$num)/($num-1);
    }
    if ($var > 0) {
	print "$var\n";
    }
    else {
	print "NONE\n";
    }
}
