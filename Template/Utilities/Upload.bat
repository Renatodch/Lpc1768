@echo off
IF EXIST F:\ (
	del "F:\*.bin"
	echo LOADING...
	xcopy ".\*.bin" "F:\"
	
	echo -----------------------------------     
	echo Succesful Compilation!, Code loaded
	echo -----------------------------------  
	
) ELSE ( echo ###############################################     
		 echo Succesful Compilation!, but need to connect USB 
		 echo ###############################################  )










