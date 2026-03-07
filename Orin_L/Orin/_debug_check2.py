import pandas as pd
df = pd.read_csv('follower_events_20260305_173834.csv', low_memory=False)
print('Events:', df['event'].value_counts().to_string())
print()

gps = df[df['event']=='GPS']
print(f'GPS rows: {len(gps)}')
print(f'Unique GPS lats: {gps["lat"].nunique()}')

# In GPS rows, does lat ever match the peer's ldr_lat?
if 'ldr_lat' in gps.columns:
    match_exact = (gps['lat'] == gps['ldr_lat']).sum()
    close = ((gps['lat'] - gps['ldr_lat']).abs() < 0.0000005).sum()
    print(f'GPS rows where lat == ldr_lat: {match_exact}/{len(gps)}')
    print(f'GPS rows where lat ~= ldr_lat (<0.5e-7): {close}/{len(gps)}')

# Check: the snapshot logger writes ldr_lat on EVERY row as last-known peer position
# The merger uses 'lat' for own GPS and 'ldr_lat' for peer, but they're DIFFERENT columns
# So the merger should be fine... let me check what's happening in merger output

merged = pd.read_csv('merged_20260305_174259.csv')
# The merged file has lat_fol (from follower's 'lat' col) and ldr_lat (from follower's 'ldr_lat' col)
# AND lat_ldr (from leader's 'lat' col)
# The plotter uses lat_fol and lat_ldr (from pick_cols schema 2: lat_ldr/lat_fol)

# Verify: in merged, does lat_fol alternate between two very different values?
lat_fol = merged['lat_fol'].values
diffs = pd.Series(lat_fol).diff().abs()
print(f'\nMerged lat_fol jumps > 10m ({10/111320:.7f} deg): {(diffs > 10/111320).sum()}')
print(f'Mean of lat_fol: {lat_fol.mean():.7f}')

# Print a section where lat_fol jumps happen
big_jumps = diffs[diffs > 10/111320].index[:5]
for idx in big_jumps:
    print(f'\nRow {idx}: lat_fol={merged.iloc[idx]["lat_fol"]:.7f}  lat_ldr={merged.iloc[idx]["lat_ldr"]:.7f}  '
          f'vn_fol={merged.iloc[idx]["vn_fol"]:.4f}  vn_ldr={merged.iloc[idx]["vn_ldr"]:.4f}')
    print(f'Row {idx-1}: lat_fol={merged.iloc[idx-1]["lat_fol"]:.7f}  lat_ldr={merged.iloc[idx-1]["lat_ldr"]:.7f}  '
          f'vn_fol={merged.iloc[idx-1]["vn_fol"]:.4f}  vn_ldr={merged.iloc[idx-1]["vn_ldr"]:.4f}')
