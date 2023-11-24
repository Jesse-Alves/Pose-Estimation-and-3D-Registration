function plot3Derrors( liste_pos3D, val_err, fig_handle)
% plot3Derrors
% arguments : liste_pos3D : matrice 3 x N
% val_err : vecteur 1 x N
% Affiche les erreurs données par val_err aux positions
% 3D données dans liste_pos3D sur la figure active ou la figure passee en
% argument
% Les couleurs varient du vert (erreur la plus faible) au rouge (erreur la
% plus élevée)
% Pour que l'affichage ait du sens, les erreurs doivent être positives

if nargin >= 3
    figure = fig_handle;
end

if (size(liste_pos3D,1) ~=3)
    fprintf(1, 'liste_pos3D must be 3 x nb_pos');
    return;
end
if (size(val_err,1) ~=1)
    fprintf(1, 'val_err must be 1 x nb_pos');
    return;
end
if (size(liste_pos3D,2) ~= size(val_err,2))
    fprintf(1, 'nb_pos in liste_pos3D must equal nb of error values in val_err');
    return;
end



    max_err = max(val_err);
     min_err = min(val_err);   
     valR = (val_err-min_err)/(max_err-min_err);
     valG = 1 - valR;
     list_color = [valR', valG', zeros(size(val_err,2), size(val_err,1))];
    scatter3((liste_pos3D(1,:))', (liste_pos3D(2,:))', (liste_pos3D(3,:))', 50, list_color, 'filled');
    


end

