#!/usr/bin/perl -w
no warnings "once";

#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
# Packages
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

#require 'Descriptive.pm';
use File::Basename;
use Getopt::Long;

#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
# Configurable Options
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

$DEBUG=0;

#$module = "ARQNetwork.LeftRouter.interfaceQueue[0]";
$module = "RoutingNetwork.Node2.interfaceQueue[1]";
$vector = "waiting time sum";


### Forward declarations

sub readCmdLine();
sub openRun($);
sub readVectorLine($);
sub closeRun($);
sub readLastValues();
sub mainLoop($$);


### MAIN ###

readCmdLine(); # sets @runs, $infilebase and $outfile
my ($numvals,$lastval) = readLastValues();
if ($outfile) {
    mainLoop($numvals,$lastval);
}

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
protsim_eval.pl -r runs | --runs=runs  infilebase outfile
protsim_eval.pl -? | --help

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
    $infilebase = shift @ARGV or die "Base name for input files expected";
    $outfile = shift @ARGV or warn "No output file name given, calculating only";
}


{ # enclosing the sub routine in a block, emulates static locals
    my %expectedVec;
    my %numval;
    my %fh;

    sub openRun($) {
	my $run = shift;

	$fh{$run} = undef;
	my $inputname = "${infilebase}".sprintf("%02u",$run).".vec";
	open($fh{$run},"$inputname") or die "Could not open $inputname: $!";
	$numval{$run} = 0;

	my ($vec, $mod, $name, $tuple);

	while (!defined($expectedVec{$run})) {
	    if (!defined($_ = readline(*{$fh{$run}}))) {
		last;
	    }

	    redo if /^\s*$/;
	    redo if /^\s*#/;

	    if (($vec, $mod, $name, $tuple) = /^\s*vector\s+(\d+)\s+\"([^\"]+)\"\s+\"([^\"]+)\"\s+(\d+)\s*$/) {
		if ($mod eq $module && $name eq $vector) {
		    die "Unexpected tuple value $tuple" if $tuple != 2;
		    $expectedVec{$run} = $vec;
		}
	    }
	}
        die "Vector $module.$vector not found for run $run" unless defined($expectedVec{$run});
	print "DEBUG: Found vector $module.$vector in run $run (#$expectedVec{$run})\n" if $DEBUG;
    }

    sub readVectorLine($) {
	my $run = shift;

	my ($vec, $t, $x1, $val);

	$vec = -1;
	while ($vec != $expectedVec{$run}) {
	    if (!defined($_ = readline(*{$fh{$run}}))) {
		return undef;
	    }

	    redo if /^\s*$/;
	    redo if /^\s*#/;
	    redo if /^\s*vector\s+/;

	    ($vec,$t,$x1,$val) = /^\s*(\d+)\s+([0-9eE.+\-]+)\s+([0-9eE.+\-]+)\s+([0-9eE.+\-]+)\s*$/
	      or die "Syntax error in file ${infilebase}".sprintf("%02u",$run).".vec";
	}

	die "Unexpected number in file ${infilebase}".sprintf("%02u",$run).".vec" if $x1 != $numval{$run}+1;
	++$numval{$run};

	return $val;
    }

    sub closeRun($) {
	close($fh{$run});
    }
}


sub readLastValues() {
    my (%newest,%val);
    my $numval = -1; # We count one to much, so start at -1

    foreach $run (@runs) {
	openRun($run);
	$newest{$run} = 0;
    }

    my $found = 0;
    do {
	foreach $run (@runs) {
	    $val{$run} = $newest{$run};
	    if (!defined($newest{$run} = readVectorLine($run))) {
		$found = 1;
	    }
	}
	++$numval;
    } until ($found);

    my $sum = 0;
    my $sqsum = 0;
    foreach $run (@runs) {
	closeRun($run);
	$sum += $val{$run};
	$sqsum += $val{$run}*$val{$run};
    }

    my $val = $sum/scalar(@runs);
    my $mean = $val/$numval;
    my $var = -1;
    if (scalar(@runs) >= 2) {
	$var = ($sqsum-$sum*$sum/scalar(@runs))/
	    ($numval*$numval*(scalar(@runs)-1)); 
    }

    print "DEBUG: Read $numval values, last value is $val\n" if $DEBUG;
    print "Mean of means: $mean\n";
    print "Sample variance of means: ";
    if ($var > 0) {
	print "$var\n";
    }
    else {
	print "NONE\n";
    }

    return ($numval, $val);
}


sub mainLoop($$) {
    my $numvals = shift;
    my $lastval = shift;

    my $totalMean = $lastval / $numvals;

    open(OUTPUT,">$outfile") or die "Could not open $outfile for writing: $!";

    foreach $run (@runs) {
	openRun($run);
    }

    my ($val,$sum);
    for (my $i=0; $i < $numvals; ++$i) {
	$sum = 0;
	foreach $run (@runs) {
	    if (!defined($val = readVectorLine($run))) { #>
		die "Unexpected end of file in ${infilebase}".sprintf("%02u",$run).".vec";
	    }
	    $sum += $val;
	}
	my $x_i = ($lastval - $sum/scalar(@runs))/($numvals-$i);

	$val = ($x_i - $totalMean)/$totalMean;
	print OUTPUT "$i $val\n"
    }

    foreach $run (@runs) {
	closeRun($run);
    }
    close(OUTPUT);
}
