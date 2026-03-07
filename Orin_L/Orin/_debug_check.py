import pandas as pd, numpy as np
df = pd.read_csv('merged_20260305_174259.csv')

vn = df['vn_fol'].values
jumps = np.abs(np.diff(vn))
big = np.where(jumps > 1.0)[0]
print(f'Big vn_fol jumps (>1 m/s): {len(big)}')
for i in big[:10]:
    print(f'  row {i}: {vn[i]:.4f} -> {vn[i+1]:.4f} (jump={jumps[i]:.4f})')
    r0 = df.iloc[i]
    r1 = df.iloc[i+1]
    print(f'    lat_fol: {r0["lat_fol"]:.7f} -> {r1["lat_fol"]:.7f}')
    print(f'    ldr_lat: {r0["ldr_lat"]:.7f} -> {r1["ldr_lat"]:.7f}')
    print(f'    ldr_vn:  {r0["ldr_vn"]:.4f} vn_fol: {r0["vn_fol"]:.4f}')

# Check if vn_fol == ldr_vn (follower reporting leader's velocity as own)
exact = (df['vn_fol'] == df['ldr_vn']).sum()
close = (np.abs(df['vn_fol'] - df['ldr_vn']) < 0.001).sum()
print(f'\nvn_fol == ldr_vn exact: {exact}/{len(df)}')
print(f'vn_fol ~= ldr_vn (<0.001): {close}/{len(df)}')

# Check follower vs ldr velocity correlation
corr = df['vn_fol'].corr(df['ldr_vn'])
print(f'Correlation vn_fol vs ldr_vn: {corr:.4f}')

# Are both position and velocity coming from same source?
print(f'\nlat_fol == ldr_lat exact: {(df["lat_fol"] == df["ldr_lat"]).sum()}/{len(df)}')
