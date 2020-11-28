function [] = write_text(filename, text)
    fid = fopen(filename, 'w');
    if fid == -1
        error('Failed to open file "%s"\n', filename);
    end
    
    fprintf(fid, '%s', text);
    fclose(fid);
end

