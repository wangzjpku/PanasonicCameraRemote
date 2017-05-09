VERSION 5.00
Object = "{648A5603-2C6E-101B-82B6-000000000014}#1.1#0"; "MSCOMM32.OCX"
Begin VB.Form Form1 
   Caption         =   "摄像机调试"
   ClientHeight    =   5175
   ClientLeft      =   60
   ClientTop       =   450
   ClientWidth     =   10500
   Icon            =   "Form1.frx":0000
   LinkTopic       =   "Form1"
   MaxButton       =   0   'False
   Picture         =   "Form1.frx":08CA
   ScaleHeight     =   5175
   ScaleWidth      =   10500
   StartUpPosition =   3  'Windows Default
   Begin VB.Frame 摄像机控制接口 
      Caption         =   "摄像机控制接口"
      Height          =   4695
      Left            =   7920
      TabIndex        =   28
      Top             =   240
      Width           =   2295
      Begin VB.CommandButton Command11 
         Caption         =   "暂停"
         Height          =   255
         Left            =   480
         TabIndex        =   32
         Top             =   3480
         Width           =   1335
      End
      Begin VB.CommandButton Command10 
         Caption         =   "录制"
         Height          =   255
         Left            =   480
         TabIndex        =   31
         Top             =   2880
         Width           =   1335
      End
      Begin VB.CommandButton Command9 
         Caption         =   "缩小"
         Height          =   255
         Left            =   480
         TabIndex        =   30
         Top             =   1560
         Width           =   1335
      End
      Begin VB.CommandButton Command8 
         Caption         =   "放大"
         Height          =   255
         Left            =   480
         TabIndex        =   29
         Top             =   960
         Width           =   1335
      End
      Begin VB.Label Label12 
         Caption         =   "镜头控制"
         Height          =   255
         Left            =   240
         TabIndex        =   34
         Top             =   480
         Width           =   735
      End
      Begin VB.Label Label11 
         Caption         =   "录像控制"
         Height          =   255
         Left            =   240
         TabIndex        =   33
         Top             =   2280
         Width           =   855
      End
   End
   Begin VB.CommandButton Command7 
      Caption         =   "接收"
      Height          =   855
      Left            =   120
      TabIndex        =   27
      Top             =   4200
      Width           =   495
   End
   Begin VB.Timer Timer2 
      Enabled         =   0   'False
      Interval        =   10
      Left            =   120
      Top             =   3720
   End
   Begin VB.Frame Frame2 
      Caption         =   "接收显示"
      Height          =   3135
      Left            =   3120
      TabIndex        =   18
      Top             =   120
      Width           =   4575
      Begin VB.CommandButton Command1 
         Caption         =   "清空"
         Height          =   735
         Left            =   4080
         TabIndex        =   20
         Top             =   2280
         Width           =   375
      End
      Begin VB.TextBox Text2 
         Height          =   2775
         Left            =   120
         MultiLine       =   -1  'True
         ScrollBars      =   2  'Vertical
         TabIndex        =   19
         Top             =   240
         Width           =   3855
      End
   End
   Begin VB.Frame Frame1 
      Caption         =   "计数与控制"
      Height          =   1695
      Left            =   600
      TabIndex        =   12
      Top             =   3360
      Width           =   7095
      Begin VB.CommandButton Command3 
         Caption         =   "退出"
         Height          =   375
         Left            =   5880
         TabIndex        =   26
         Top             =   1200
         Width           =   735
      End
      Begin VB.CommandButton Command6 
         BackColor       =   &H00E0E0E0&
         Caption         =   "自动发送"
         Height          =   375
         Left            =   3240
         MaskColor       =   &H8000000F&
         TabIndex        =   25
         Top             =   720
         Width           =   1455
      End
      Begin VB.TextBox Text3 
         Height          =   270
         Left            =   6120
         TabIndex        =   24
         Text            =   "1000"
         Top             =   720
         Width           =   735
      End
      Begin VB.Timer Timer1 
         Left            =   4920
         Top             =   1200
      End
      Begin VB.CommandButton Command2 
         Caption         =   "手动发送"
         Height          =   375
         Left            =   600
         TabIndex        =   22
         Top             =   720
         Width           =   1455
      End
      Begin VB.TextBox Text1 
         Height          =   375
         Left            =   120
         TabIndex        =   21
         Top             =   240
         Width           =   6855
      End
      Begin VB.CommandButton Command5 
         Caption         =   "清空计数"
         Height          =   375
         Left            =   3240
         TabIndex        =   17
         Top             =   1200
         Width           =   975
      End
      Begin VB.Label Label10 
         Caption         =   "自动发送周期："
         Height          =   255
         Left            =   4800
         TabIndex        =   23
         Top             =   840
         Width           =   1335
      End
      Begin VB.Label Label6 
         Caption         =   "Label6"
         Height          =   255
         Left            =   2520
         TabIndex        =   16
         Top             =   1320
         Width           =   735
      End
      Begin VB.Label Label9 
         Caption         =   "接收："
         Height          =   255
         Left            =   1800
         TabIndex        =   15
         Top             =   1320
         Width           =   615
      End
      Begin VB.Label Label7 
         Caption         =   "Label7"
         Height          =   255
         Left            =   960
         TabIndex        =   14
         Top             =   1320
         Width           =   735
      End
      Begin VB.Label Label8 
         Caption         =   "发送："
         Height          =   255
         Left            =   240
         TabIndex        =   13
         Top             =   1320
         Width           =   615
      End
   End
   Begin VB.CommandButton Command4 
      Caption         =   "Command4"
      Height          =   375
      Left            =   1680
      TabIndex        =   11
      Top             =   600
      Width           =   975
   End
   Begin VB.ComboBox Combo5 
      Height          =   300
      ItemData        =   "Form1.frx":1194
      Left            =   1080
      List            =   "Form1.frx":11A7
      TabIndex        =   9
      Text            =   "8"
      Top             =   2280
      Width           =   1815
   End
   Begin VB.ComboBox Combo4 
      Height          =   300
      ItemData        =   "Form1.frx":11BA
      Left            =   1080
      List            =   "Form1.frx":11C7
      TabIndex        =   5
      Text            =   "1"
      Top             =   2760
      Width           =   1815
   End
   Begin VB.ComboBox Combo3 
      Height          =   300
      ItemData        =   "Form1.frx":11D6
      Left            =   1080
      List            =   "Form1.frx":11E3
      TabIndex        =   4
      Text            =   "N"
      Top             =   1800
      Width           =   1815
   End
   Begin VB.ComboBox Combo2 
      Height          =   300
      ItemData        =   "Form1.frx":11F0
      Left            =   1080
      List            =   "Form1.frx":1218
      TabIndex        =   3
      Text            =   "115200"
      Top             =   1200
      Width           =   1815
   End
   Begin VB.PictureBox Picture1 
      BackColor       =   &H000000FF&
      FillStyle       =   2  'Horizontal Line
      ForeColor       =   &H00000000&
      Height          =   255
      Left            =   1080
      ScaleHeight     =   195
      ScaleWidth      =   195
      TabIndex        =   1
      Top             =   720
      Width           =   255
   End
   Begin VB.ComboBox Combo1 
      Height          =   300
      ItemData        =   "Form1.frx":1269
      Left            =   1080
      List            =   "Form1.frx":128B
      TabIndex        =   0
      Text            =   "COM1"
      Top             =   240
      Width           =   1815
   End
   Begin MSCommLib.MSComm MSComm1 
      Left            =   0
      Top             =   3000
      _ExtentX        =   1005
      _ExtentY        =   1005
      _Version        =   393216
      CommPort        =   5
      DTREnable       =   -1  'True
   End
   Begin VB.Label Label5 
      Caption         =   "数据位"
      Height          =   375
      Left            =   480
      TabIndex        =   10
      Top             =   2280
      Width           =   615
   End
   Begin VB.Label Label4 
      Caption         =   "停止位"
      Height          =   375
      Left            =   480
      TabIndex        =   8
      Top             =   2760
      Width           =   615
   End
   Begin VB.Label Label3 
      Caption         =   "校验位"
      Height          =   255
      Left            =   480
      TabIndex        =   7
      Top             =   1800
      Width           =   615
   End
   Begin VB.Label Label2 
      Caption         =   "比特率"
      Height          =   255
      Left            =   480
      TabIndex        =   6
      Top             =   1200
      Width           =   615
   End
   Begin VB.Label Label1 
      Caption         =   "串 口"
      Height          =   255
      Left            =   600
      TabIndex        =   2
      Top             =   240
      Width           =   495
   End
End
Attribute VB_Name = "Form1"
Attribute VB_GlobalNameSpace = False
Attribute VB_Creatable = False
Attribute VB_PredeclaredId = True
Attribute VB_Exposed = False
Private Declare Sub Sleep Lib "kernel32" (ByVal dwMilliseconds As Long)


Private Sub Command1_Click()
'Text1.Text = ""
Text2.Text = ""
End Sub

Private Sub Command10_Click()
Dim Data(0 To 3) As Byte
Data(0) = "&HFF"
Data(1) = "&H00"
Data(2) = "&H01"
Data(3) = "&H00"


On Error GoTo aaa
If MSComm1.PortOpen = True Then

'Timer1.Enabled = False
If Text1.Text = "" Then
Text1.Text = "NULL"
Else
MSComm1.Output = Data
Label7.Caption = Label7.Caption + Len(Trim(Text1.Text))
End If
Else
MsgBox ("请打开COM口")
Exit Sub
End If

Call Receive_Click
Exit Sub
aaa:
End Sub

Private Sub Command11_Click()
Dim Data(0 To 3) As Byte
Data(0) = "&HFF"
Data(1) = "&H00"
Data(2) = "&H00"
Data(3) = "&H00"


On Error GoTo aaa
If MSComm1.PortOpen = True Then

'Timer1.Enabled = False
If Text1.Text = "" Then
Text1.Text = "NULL"
Else
MSComm1.Output = Data
Label7.Caption = Label7.Caption + Len(Trim(Text1.Text))
End If
Else
MsgBox ("请打开COM口")
Exit Sub
End If

Call Receive_Click
Exit Sub
aaa:
End Sub

Private Sub Command2_Click()
On Error GoTo aaa
If MSComm1.PortOpen = True Then

'Timer1.Enabled = False
If Text1.Text = "" Then
Text1.Text = "NULL"
Else
MSComm1.Output = Trim(Text1.Text)
Label7.Caption = Label7.Caption + Len(Trim(Text1.Text))
End If
Else
MsgBox ("请打开COM口")
Exit Sub
End If

Call Receive_Click
Exit Sub
aaa:
End Sub

Private Sub Receive_Click()
Dim buf$
Dim A As String
Dim bytreceive() As Byte
Dim I As Integer
On Error GoTo aaa
'buf = ""
delay (20)
buf = MSComm1.Input
If Len(buf) = 0 Then
Text2.Text = "NULL"
Else
A = Text2.Text
bytreceive() = buf
Text2.Text = A & buf & " "
'For I = 0 To UBound(bytreceive)
'A = A&Hex(bytreceive(i))
'text2.Text =text2.text&hex(bytreceive(i))&" "
End If
Label6.Caption = Label6.Caption + Len(buf)
Exit Sub
aaa:
End Sub

Private Sub combo1_Click()
On Error GoTo aaa
If MSComm1.PortOpen = True Then

MSComm1.PortOpen = False
MSComm1.CommPort = Mid(Combo1.Text, 4)
'MSComm1.PortOpen = False
MSComm1.PortOpen = True
'Call Command4_Click
Command4.Caption = "关闭"
Text1.Locked = False
Picture1.BackColor = &HFF00&
Combo2.Enabled = False
Combo3.Enabled = False
Combo4.Enabled = False
Combo5.Enabled = False
'MsgBox ("COM口已经正确打开")
Else


MSComm1.CommPort = Mid(Combo1.Text, 4)
'MSComm1.PortOpen = False
MSComm1.PortOpen = True
'Call Command4_Click
Text1.Locked = False
Picture1.BackColor = &HFF00&
Combo2.Enabled = False
Combo3.Enabled = False
Combo4.Enabled = False
Combo5.Enabled = False
Command4.Caption = "关闭"
'MsgBox ("COM口已经正确打开")
'MSComm1.Output = "接口测试"
End If


Exit Sub

aaa:
Text1.Locked = True
Picture1.BackColor = &HFF&
Combo2.Enabled = True
Combo3.Enabled = True
Combo4.Enabled = True
Combo5.Enabled = True
Command4.Caption = "打开"
MsgBox ("请选择正确的COM口")
'MSComm1.PortOpen = False

End Sub

Public Sub delay(msec As Long)
On Error Resume Next
Dim tstart As Single
tstart = Timer
While (Timer - tstart) < (msec / 1000)
DoEvents
Wend
End Sub

Private Sub Command6_Click()
'Timer1.Enabled = False
If Command6.Caption = "自动发送" Then

Timer1.Interval = Trim(Text3.Text)
Timer1.Enabled = True
Command6.Caption = "关闭自动发送"
Else
Timer1.Enabled = False
Command6.Caption = "自动发送"
End If
End Sub

Private Sub Command3_Click()
End
End Sub

Private Sub Command4_Click()
On Error GoTo aaa
If Command4.Caption = "关闭" Then

MSComm1.PortOpen = False
Command4.Caption = "打开"
Picture1.BackColor = &HFF&
Combo2.Enabled = True
Combo3.Enabled = True
Combo4.Enabled = True
Combo5.Enabled = True
Else
MSComm1.Settings = Combo2.Text & "," & Combo3.Text & "," & Combo5.Text & "," & Combo4.Text
MSComm1.PortOpen = True
Command4.Caption = "关闭"
Picture1.BackColor = &HFF00&
Combo2.Enabled = False
Combo3.Enabled = False
Combo4.Enabled = False
Combo5.Enabled = False
End If
Exit Sub
aaa:
MsgBox ("请选择正确的COM口")
End Sub

Private Sub Command5_Click()
Label6.Caption = 0
Label7.Caption = 0
End Sub

Private Sub Command7_Click()
If Command7.Caption = "接收" Then
Timer2.Enabled = True

Call Timer2_Timer
Command7.Caption = "关闭接收"
Else
Command7.Caption = "接收"
Timer2.Enabled = False
End If
End Sub




Private Sub Command8_Click()

Dim Data(0 To 3) As Byte
Data(0) = "&HFF"
Data(1) = "&H01"
Data(2) = "&H00"
Data(3) = "&H10"

Dim Data2(0 To 3) As Byte
Data2(0) = "&HFF"
Data2(1) = "&H01"
Data2(2) = "&H00"
Data2(3) = "&H30"

On Error GoTo aaa
If MSComm1.PortOpen = True Then

'Timer1.Enabled = False
If Text1.Text = "" Then
Text1.Text = "NULL"
Else
MSComm1.Output = Data
Sleep 500
MSComm1.Output = Data2
End If
Else
MsgBox ("请打开COM口")
Exit Sub
End If

Call Receive_Click
Exit Sub
aaa:
End Sub



Private Sub Command9_Click()
Dim Data(0 To 3) As Byte
Data(0) = "&HFF"
Data(1) = "&H01"
Data(2) = "&H00"
Data(3) = "&H69"

Dim Data2(0 To 3) As Byte
Data2(0) = "&HFF"
Data2(1) = "&H01"
Data2(2) = "&H00"
Data2(3) = "&H30"

On Error GoTo aaa
If MSComm1.PortOpen = True Then

'Timer1.Enabled = False
If Text1.Text = "" Then
Text1.Text = "NULL"
Else
MSComm1.Output = Data
Sleep 500
MSComm1.Output = Data2

End If
Else
MsgBox ("请打开COM口")
Exit Sub
End If

Call Receive_Click
Exit Sub
aaa:
End Sub

Private Sub Form_Load()
Label7.Caption = 0
Label6.Caption = 0
Timer1.Enabled = False
'Option2.Caption = "十六进制发送"
'Option1.Caption = "字符串发送"
On Error GoTo aaa
MSComm1.Settings = "115200,n,8,1"


MSComm1.CommPort = 1
MSComm1.PortOpen = True
Text1.Locked = False
Text2.Locked = True
Picture1.BackColor = &HFF00&
Command4.Caption = "关闭"
Combo2.Enabled = False
Combo3.Enabled = False
Combo4.Enabled = False
Combo5.Enabled = False
Exit Sub
aaa:
Text1.Locked = True
Picture1.BackColor = &HFF&
Combo2.Enabled = True
Combo3.Enabled = True
Combo4.Enabled = True
Combo5.Enabled = True
MsgBox ("请选择正确的COM口")
Command4.Caption = "打开"

End Sub

Private Sub Option2_Click()
If Option2.Value = True Then
End If
End Sub

Private Sub Timer1_Timer()
Call Command2_Click
End Sub

Private Sub Timer2_Timer()
Call Receive_Click
End Sub



