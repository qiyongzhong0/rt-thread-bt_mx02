from building import *

cwd = GetCurrentDir()
path = [cwd+'/inc']
src  = Glob('src/*.c')
 
group = DefineGroup('bt_mx02', src, depend = ['PKG_USING_BT_MX02'], CPPPATH = path)

Return('group')