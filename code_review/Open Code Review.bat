@echo off
rem Double-click to open the live Navi Bot code review in your browser.
rem The report updates as you save files. Close this window to stop it.
title Navi Bot code review
cd /d "%~dp0.."
python "%~dp0review.py" serve %*
if errorlevel 1 pause
