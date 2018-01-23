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

@modules = ('ARQNetwork.LeftNode[0].GoBackNReceiver1',
	    'ARQNetwork.RightNode[0].GoBackNReceiver1');
@scalars = ('goodput','throughput');

%tTable = ( 1 => 12.706, 2 => 4.303, 3 => 3.182, 4 => 2.776, 5 => 2.571,
	    6 =>  2.447, 7 => 2.365, 8 => 2.306, 9 => 2.262, 10 => 2.228,
	    11 => 2.201, 12 => 2.179, 13 => 2.160, 14 => 2.145, 15 => 2.131,
	    16 => 2.120, 17 => 2.110, 18 => 2.101, 19 => 2.093, 20 => 2.086,
	    21 => 2.080, 22 => 2.074, 23 => 2.069, 24 => 2.064, 25 => 2.060,
	    26 => 2.056, 27 => 2.052, 28 => 2.048, 29 => 2.045 );


### Forward Declarations ###

sub readCmdLine();
sub openFile($);
sub closeFile();
sub readScalarLines();
sub mainLoop();


### MAIN ###

foreach my $i (@modules) {
    $modules{$i} = 1;
}
foreach my $i (@scalars) {
    $scalars{$i} = 1;
}

readCmdLine(); # sets @runs, $infilebase and $outfile
mainLoop();

exit(0);


### Sub routines ###

sub readCmdLine() {
    my $help = 0;
    my @lruns = ();
    my %lruns = ();

    Getopt::Long::Configure ("bundling");
    GetOptions('runs|r=s' => \@lruns,
	       'help|?'=>\$help ) or $help = 2;

    if ($help) {
	print <<EOF;
protsim_arq_eval.pl -r runs | --runs=runs  infilebase
protsim_arq_eval.pl -? | --help

EOF
	exit 1 if $help == 2;
	exit 0;
    }


    @lruns = split(/,/,join(',',@lruns));
    print "DEBUG: lruns: ",join("\n",@lruns) if $DEBUG;
    print "\n" if $DEBUG;

    foreach my $run (@lruns) {
	if ($run =~ /^\s*(\d+)-(\d+)\s*$/) {
	    my $lower = $1;
	    my $higher = $2;
	    die "Lower bound of range bigger than upper bound in runs specification" if $lower > $higher;
	    my $i;
	    for ($i = $lower; $i <= $higher; ++$i) {
		$lruns{$i} = 1;
	    }
	} else {
	    die "Numeric run number expected" unless $run =~ /^\s*(\d+)\s*$/;

	    $lruns{$1} = 1;
	}
    }

    @runs = sort {$a <=> $b} keys %lruns;
    foreach my $run (@runs) {
	if ($run < 10) {
	    $run = "0$run";
	}
    }

    $infilebase = shift @ARGV or die "Base name for input files expected";
}


{ # enclosing the sub routine in a block, emulates static locals
    my $numval = -1;
    local *INPUT;

    sub openFile($) {
	my $name = shift or die "No filename given";
	open(INPUT,"$name") or die "Could not open $name: $!";
    }

    sub closeFile() {
	close(INPUT);
    }

    sub readScalarLines() {
	my ($mod, $vec, $val, $batch);
	my %values;

	while (<INPUT>) {
	    if (($mod, $sca, $val) = /^\s*scalar\s+"([^"]*)"\s+"([^"]*)"\s+([0-9eE.+\-]+)\s+$/) {
		if (defined $modules{$mod} && defined $scalars{$sca}) {
		    $values{$mod}{$sca} = $val;
		}
	    }
	}

	return \%values;
    }
}




sub mainLoop() {
    my %values;

    foreach $run (@runs) {
	openFile("${infilebase}".sprintf("%02u",$run).".sca");
	$values{$run} = readScalarLines();
	closeFile();
    }

    printf "%-70s\t%-8s\t%s\n", "","Mean","+/- (95% Conf.)";
    foreach my $scalar (@scalars) {
	my ($totalNum, $totalSum,$totalSqSum);
	$totalNum = $totalSum = $totalSqSum = 0;

	foreach my $module (@modules) {
	    my ($num,$sum,$sqsum);
	    $num = $sum = $sqsum = 0;

	    foreach my $run (@runs) {
		die "$module.$scalar missing in run $run"
		  if !defined $values{$run}{$module}{$scalar};

		my $val = $values{$run}{$module}{$scalar};
		print "$run $module.$scalar: $val\n" if $DEBUG;
		$num++;
		$sum += $val;
		$sqsum += $val*$val;
	    }

	    print "Number: $num\n" if $DEBUG;
	    my $mean = $sum/$num;
	    printf "%-70s\t%8.0f\t", "$module.$scalar:", $mean;
	    my $var = -1;
	    if ($num >= 2) {
		$var = ($sqsum-$sum*$sum/$num)/($num-1);
	    }
	    print "Variance: $var\n" if $DEBUG;
	    my $conf = -1;
	    if ($var > 0) {
		if ($num >= 30) { $conf = 1.962*sqrt($var/$num); }
		elsif (defined $tTable{$num-1}) {
		    $conf = $tTable{$num-1}*sqrt($var/$num);
		}
	    }

	    if ($conf > 0) {
		printf "%8.0f\n", $conf;
	    } else {
		print "N/A\n";
	    }
	}

	foreach my $run (@runs) {
	    my ($num,$sum);
	    $num = $sum = 0;
	    foreach my $module (@modules) {
		die "$module.$scalar missing in run $run"
		  if !defined $values{$run}{$module}{$scalar};

		my $val = $values{$run}{$module}{$scalar};
		print "$run $module.$scalar: $val\n" if $DEBUG;

		$num++;
		$sum += $val;
	    }
	    my $mean = $sum / $num;
	    $totalNum++;
	    $totalSum += $mean;
	    $totalSqSum += $mean*$mean;
	}

	print "Number: $totalNum\n" if $DEBUG;
	my $mean = $totalSum/$totalNum;
	printf "%-70s\t%8.0f\t", "NETWORK.$scalar:", $mean;
	my $var = -1;
	if ($totalNum >= 2) {
	    $var = ($totalSqSum-$totalSum*$totalSum/$totalNum)/($totalNum-1);
	}
	print "Variance: $var\n" if $DEBUG;
	my $conf = -1;
	if ($var > 0) {
	    if ($totalNum >= 30) { $conf = 1.962*sqrt($var/$totalNum); }
	    elsif (defined $tTable{$totalNum-1}) {
		$conf = $tTable{$totalNum-1}*sqrt($var/$totalNum);
	    }
	}

	if ($conf > 0) {
	    printf "%8.0f\n", $conf;
	} else {
	    print "N/A\n";
	}
    }
}
