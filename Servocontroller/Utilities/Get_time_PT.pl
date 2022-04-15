#!/usr/local/bin/perl
#use warnings;
#use diagnostics;
#use POSIX qw(strftime);
use Cwd qw();

if (!$ARGV[0] || !$ARGV[1]){
	die "Parameter wrong!!!";
}

print "Begin to generate files!\n";

my $tTime= localtime(time);
($sec,$min,$hour,$mday,$mon,$year_off,$wday,$yday,$isdat) = localtime;
$year_off += 1900;
$mon += 1;

#my $timeStr = strftime "%Y-%m-%d %H:%M:%S%", localtime;
my $timeStr;
#print "Current System Time: $year_off/$mon/$mday $hour:$min:$sec \n";
$timeStr = sprintf "%04d/%02d/%02d %02d:%02d:%02d", $year_off, $mon, $mday, $hour, $min, $sec;
print "Current System Time: $timeStr.\n";

#Get LIB generate time from oa_verno.obj
#Save it in $get_lib_time

#open RF, '..//lib//oa_verno.obj';
open RF, 'lib//oa_verno.obj';
my $len_read = 1024*4;
my $len = sysread RF, $buf_verno, $len_read;  
my $get_lib_time;


if($buf_verno =~ m/\d{4}\/\d{1,2}\/\d{1,2} \d{1,2}:\d{1,2}/i)
{
	print("Find a time express. ");
	$get_lib_time = $&;
	print("Matched: <$get_lib_time>\n");
}
else
{

}
close RF;

my $path = Cwd::abs_path();
my @values = split('/', $path);
$k=0;
foreach my $val (@values) {
   $k++;
  }

print("Version: \"$values[k-1]\".\n");

#source file text
my $srcFileName = $ARGV[0];
my $srcFileTxt = "#include \"build_time.h\"\n";
$srcFileTxt .= "/*-----Do not modify, get_build_date_time same with build_date_time() for validity check------*/\n";
$srcFileTxt .= "char* get_build_date_time(void)\n";
$srcFileTxt .= "{\n";
$srcFileTxt .= "\tstatic char build_bin_date_time_str[] = \"$timeStr\";\n";
$srcFileTxt .= "\treturn build_bin_date_time_str;\n";
$srcFileTxt .= "}\r\n";

$srcFileTxt .= "/*-----Do not modify, oa_lib_build_date_time same with build_date_time() for validity check------*/\n";
$srcFileTxt .= "char* get_lib_build_date_time(void)\n";
$srcFileTxt .= "{\n";
$srcFileTxt .= "\tstatic char build_lib_date_time_str[] = \"$get_lib_time\";\n";
$srcFileTxt .= "\treturn build_lib_date_time_str;\n";
$srcFileTxt .= "}\r\n";
$srcFileTxt .= "\r\n";

$srcFileTxt .= "/*-----Do not modify, oa_lib_build_date_time same with build_date_time() for validity check------*/\n";
$srcFileTxt .= "char* get_version(void)\n";
$srcFileTxt .= "{\n";
$srcFileTxt .= "\tstatic char version_soft_str[] = \"$values[k-1]\";\n";
$srcFileTxt .= "\treturn version_soft_str;\n";
$srcFileTxt .= "}\r\n";
$srcFileTxt .= "\r\n";

$srcFileTxt .= "/********************FIN*****************/";
$srcFileTxt .= "\r\n";

#header file text
my $incFileName = $ARGV[1];
$incFileTxt .= "char* get_build_date_time(void);\r\n";
$incFileTxt .= "char* get_lib_build_date_time(void);\r\n";
$incFileTxt .= "char* get_version(void);\r\n";
$srcFileTxt .= "\r\n";
$incFileTxt .= "/********************FIN*****************/";
$incFileTxt .= "\r\n";

#generate source file for system time
open(FILE_HANDLE, ">$srcFileName") or die "cannot open $srcFileName\n";
print FILE_HANDLE &copyright_file_header();
print FILE_HANDLE $srcFileTxt;
close(FILE_HANDLE);

#generate header file for system time
open(FILE_HANDLE, ">$incFileName") or die "cannot open $incFileName\n";
print FILE_HANDLE &copyright_file_header();
print FILE_HANDLE $incFileTxt;
close(FILE_HANDLE);

print "$srcFileName and $incFileName generated Finished!\n";




sub copyright_file_header
{
    my $template = <<"__TEMPLATE";
/*****************************************************************************
 * Fecha y Hora de Compilacion de Proyecto:
 * ---------------------------------------
 * **** TELCOMIP SAC **** $dir
 *****************************************************************************/

__TEMPLATE

	return $template;
}

