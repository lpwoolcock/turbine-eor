function [text] = read_text(filename)
    fid = fopen(filename);
    if fid == -1
        error('Failed to open file "%s"\n', filename);
    end
    
    text = fread(fid, '*char').';
    fclose(fid);
end

