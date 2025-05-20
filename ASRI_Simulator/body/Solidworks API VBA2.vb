Option Explicit

Sub main()

    Dim swApp                   As SldWorks.SldWorks
    Dim swModel                 As SldWorks.ModelDoc2
    Dim swModelExt              As SldWorks.ModelDocExtension
    Dim swSelMgr                As SldWorks.SelectionMgr
    Dim swComp                  As SldWorks.Component2
    Dim nStatus                 As Long
    Dim vMassProp               As Variant
    Dim i                       As Long
    Dim nbrSelections           As Long

    Set swApp = Application.SldWorks
    Set swModel = swApp.ActiveDoc
    Set swModelExt = swModel.Extension
    Set swSelMgr = swModel.SelectionManager
   

    nbrSelections = swSelMgr.GetSelectedObjectCount2(-1)

    If nbrSelections = 0 Then
        Debug.Print "Please select one or more components and rerun the macro."
        Exit Sub
     End If

    nbrSelections = nbrSelections - 1    
    Debug.Print "Getting mass properties for components: "
    For i = 0 To nbrSelections
         Set swComp = swSelMgr.GetSelectedObject6(i + 1, -1)
         Debug.Print "  " & swComp.Name2
    Next

    vMassProp = swModelExt.GetMassProperties2(1, nStatus, True)

    If Not IsEmpty(vMassProp) Then

        Debug.Print "Center of mass:"
        Debug.Print "  X-coordinate = " & vMassProp(0)
        Debug.Print "  Y-coordinate = " & vMassProp(1)
        Debug.Print "  Z-coordinate = " & vMassProp(2)
        Debug.Print "Mass = " & vMassProp(5)
        Debug.Print "Moments of inertia taken at the center of mass:"
        Debug.Print "  Lxx = " & vMassProp(6)
        Debug.Print "  Lyy = " & vMassProp(7)
        Debug.Print "  Lzz = " & vMassProp(8)

    End If

End Sub