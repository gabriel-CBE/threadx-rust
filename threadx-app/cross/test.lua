local function find_vscode_settings(bufname)
  local settings = {}
  local found_dirs = vim.fs.find({ '.vscode' }, { upward = true, path = vim.fs.dirname(bufname), type = 'directory' })
  if vim.tbl_isempty(found_dirs) then
    return settings
  end
  local vscode_dir = found_dirs[1]
  local results = vim.fn.glob(vim.fs.joinpath(vscode_dir, 'settings.json'), true, true)
  if vim.tbl_isempty(results) then
    return settings
  end
  local content = os.read_file(results[1])
  return content 
end
