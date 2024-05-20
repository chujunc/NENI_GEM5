#!/usr/bin/perl

@benches = ('blackscholes', 'bodytrack', 'canneal', 'dedup', 'facesim',
            'ferret', 'fluidanimate', 'freqmine', 'streamcluster',
            'swaptions', 'vips', 'x264', 'rtview' );

$inputsets[1] = "test";
$inputsets[2] = "simdev";
$inputsets[3] = "simsmall";
$inputsets[4] = "simmedium";
$inputsets[5] = "simlarge";

# check for proper input
if( scalar(@ARGV) != 2 ) {
  printUsage();
  exit;
}

# grab input
$bench = $ARGV[0];
$found = 0;
foreach $b (@benches) {
  if( $bench eq $b ) {
    $found = 1;
  }
}
if( ! $found ) {
  printBenches();
}

# set up the input dir
$inputdir = "/parsec/install/inputs/$bench/";

$numthreads = int($ARGV[1]);

# open the input sets file:
open( INFILE, "./inputsets.txt" );

$found = 0;
while( ! $found && ($line = <INFILE>) ) {
  if( $line =~ m/$bench/ ) {
    $found = 1;
    chomp( $line );
    @tokens = split(/\;/,$line);

    # replace the input dir and numthreads
    for( $i = 0; $i < scalar(@tokens); $i++ ) {
      while( $tokens[$i] =~ m/<inputdir>/ ) {
        $tokens[$i] =~ s/<inputdir>/$inputdir/;
      }
      $tokens[$i] =~ s/<nthreads>/$numthreads/;
      # print "$tokens[$i]\n";
    }

    for( $i = 1; $i < scalar(@inputsets); $i++ ) {
      $inp = $inputsets[$i];
      $filename = $bench."_".$numthreads."c_".$inp.".rcS";
      open( FILE, "> $filename" ) or die "File problem\n";
      print FILE "#!/bin/sh\n\n";
      print FILE "# File to run the $bench benchmark\n\n";
      if( $bench eq "vips" ) {
        print FILE "export IM_CONCURRENCY=$numthreads\n";
      }
      print FILE "cd /parsec/install/bin\n";
      print FILE "/sbin/m5 switchcpu\n";
      print FILE "/sbin/m5 dumpstats\n";
      print FILE "/sbin/m5 resetstats\n";
      print FILE "./$bench $tokens[$i]\n";
      print FILE "echo \"Done :D\"\n";
      print FILE "/sbin/m5 exit\n";
      print FILE "/sbin/m5 exit\n";
      close( FILE );
    }
  }
}

# close the input file:
close( INFILE );

########################
# Methods
########################
sub printUsage() {
  print "\tWrites the .rcS files for specified parsec benchmark\n";
  print " with specified number of threads\n";
  print "Usage:\n";
  print "\t./writescripts.pl <benchmark> <nthreads>\n";
}

sub printBenches() {
  print "Valid PARSEC Benchmarks:\n";
  foreach $b (@benches) {
    print "\t$b\n";
  }
}
