Dim swApp
Set swApp = CreateObject("SldWorks.Application")

Dim filePath
filePath = InputBox("Part_File_Path")

Dim docSpec
Set docSpec = swApp.GetOpenDocSpec(filePath)
docSpec.ReadOnly = True
docSpec.Silent = True

Dim swModel
Set swModel = swApp.OpenDoc7(docSpec)

Dim swMassPrps
Set swMassPrps = swModel.Extension.CreateMassProperty()

MsgBox "Mass: " & swMassPrps.Mass & vbLf & "Volume: " & swMassPrps.Volume & vbLf & "Surface area: " & swMassPrps.SurfaceArea
MsgBox "Mass: " & swMassPrps.Mass
MsgBox "COM:  " & swMassPrps(0)
MsgBox "MOIx: " & swMassPrps(6)
MsgBox "MOIx: " & swMassPrps(7)
MsgBox "MOIx: " & swMassPrps(8)
swApp.CloseDoc swModel.GetTitle()
