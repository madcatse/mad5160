Dim BoardName
Dim NetName
If IsObject(WScript) Then
	BoardName = WScript.Arguments(0)
	NetName = WScript.Arguments(1)	
Else
	' Linux
	BoardName = ScriptHelper.Arguments.item(3)
	' RHEL6 - need to remove quotes
	BoardName = Replace(BoardName, Chr(34), "")
	NetName = ScriptHelper.Arguments.item(4)
	NetName = Replace(NetName, Chr(34), "")
End If

Set App = GetApp("HyperLynx.HLApplication", "86")
Set Design = App.Design

Dim fso: set fso = CreateObject("Scripting.FileSystemObject")
Dim BoardPath : BoardPath = fso.BuildPath(fso.GetAbsolutePathName(".."), BoardName)
If Not fso.FileExists(BoardPath) Then
	'SerDes
	BoardPath = fso.BuildPath(fso.GetAbsolutePathName("../.."), BoardName)
End If


If Design Is Nothing Then
	' Nothin is loaded
	Dim LoadDesign : LoadDesign = MsgBox ("No design loaded in HyperLynx.  Would you like to load " + BoardName + "?", vbYesNo + vbQuestion, "Design not loaded")
	Select Case LoadDesign
	Case vbYes
		App.Visible = True
		LoadBoard(BoardPath)		
	Case vbNo
		App.Exit
		Wscript.Quit
	End Select	
Else
	intCompare = StrComp(App.FileName, BoardPath, 1)
	If Not intCompare = 0 Then
		' Different design loaded
		Dim ChangeDesign : ChangeDesign = MsgBox ("Different design loaded HyperLynx: " + App.FileName + "  Would you like to load " + BoardPath + "?", vbYesNo + vbQuestion, "Different design loaded")
		Select Case ChangeDesign
		Case vbYes			
			LoadBoard(BoardPath)		
		Case vbNo
			Wscript.Quit
		End Select	
	End If	
End If

'Select net
Set Net = App.Design.FindNet(NetName)
If Net Is Nothing Then 
	MsgBox("Net not found: " + Net)
Else	
	App.Design.SelectNet(NetName)
	
	' Zoom in around the net
	Dim Left
	Dim Bottom
	Dim Right
	Dim Top
	GetZoomRect(Net)
	Dim AssocNet
	For Each AssocNet in Net.RelatedNets
		GetZoomRect(AssocNet)
	Next

	' Rounding error
	Left = Left - 0.001 
	Right = Right + 0.001	

	Dim Left2
	Dim Bottom2
	Dim Right2
	Dim Top2    
	App.Design.Viewer.GetDesignExtents Left2, Bottom2, Right2, Top2

	If Left < Left2 Then
		Left = Left2
	End If

	If Right > Right2 Then
		Right = Right2
	End If

	App.Design.Viewer.ZoomTo Left, Bottom, Right, Top
End If	


Function GetZoomRect(Net)
	Dim Seg	
	For Each Seg in Net.Segments
		If Seg.X1 <= Seg.X2 Then
			If Left = Empty or Left > Seg.X1 Then
				Left = Seg.X1		
			End If
			If Right = Empty or Right < Seg.X2 Then
				Right = Seg.X2
			End If
		Else
			If Left = Empty or Left > Seg.X2 Then
				Left = Seg.X2		
			End If
			If Right = Empty or Right < Seg.X1 Then
				Right = Seg.X1
			End If		
		End If
		
		If Seg.Y1 <= Seg.Y2 Then
			If Bottom = Empty or Bottom > Seg.Y1 Then
				Bottom = Seg.Y1		
			End If
			If Top = Empty or Top < Seg.Y2 Then
				Top = Seg.Y2
			End If
		Else
			If Bottom = Empty or Bottom > Seg.Y2 Then
				Bottom = Seg.Y2		
			End If
			If Top = Empty or Top < Seg.Y1 Then
				Top = Seg.Y1
			End If		
		End If
	Next
End Function


Function LoadBoard(BoardPath)	
	If fso.FileExists(BoardPath) Then						
		App.OpenFile(BoardPath)		
	Else
		MsgBox("Board not found: " + BoardPath)
		App.Exit
	End If
End Function


Function GetApp(AppName, Version)
	On error resume next
		Set GetApp = GetObject("", AppName + "." + Version)
		If err.number <> 0 Then
			Set GetApp = GetObject("", AppName)
		End If
	On error goto 0	
End Function
