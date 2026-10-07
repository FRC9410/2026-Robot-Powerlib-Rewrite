@echo off
powershell -ExecutionPolicy Bypass -File "%~dp0generate-subsystem.ps1" -UpdateSubsystems %*

