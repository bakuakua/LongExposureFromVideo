<<<<<<< HEAD
function [Offset, SearchRegion] = updatesearch(sz, MotionVector, SearchRegion, Offset, pos)
% Function to update Search Region for SAD and Offset for Translate
 
  % check bounds
  A_i = Offset - MotionVector;
  AbsTemplate = pos.template_orig - A_i;
  SearchTopLeft = AbsTemplate - pos.search_border;
  SearchBottomRight = SearchTopLeft + (pos.template_size + 2*pos.search_border);
 
  inbounds = all([(SearchTopLeft >= [1 1]) (SearchBottomRight <= fliplr(sz))]);
 
  if inbounds
      Mv_out = MotionVector;
  else
      Mv_out = [0 0];
  end
 
  Offset = Offset - Mv_out;
  SearchRegion = SearchRegion + Mv_out;
 
end % function updatesearch
=======
function [Offset, SearchRegion] = updatesearch(sz, MotionVector, SearchRegion, Offset, pos)
% Function to update Search Region for SAD and Offset for Translate
 
  % check bounds
  A_i = Offset - MotionVector;
  AbsTemplate = pos.template_orig - A_i;
  SearchTopLeft = AbsTemplate - pos.search_border;
  SearchBottomRight = SearchTopLeft + (pos.template_size + 2*pos.search_border);
 
  inbounds = all([(SearchTopLeft >= [1 1]) (SearchBottomRight <= fliplr(sz))]);
 
  if inbounds
      Mv_out = MotionVector;
  else
      Mv_out = [0 0];
  end
 
  Offset = Offset - Mv_out;
  SearchRegion = SearchRegion + Mv_out;
 
end % function updatesearch
>>>>>>> caf6b9d48dfa566cb6750a628deb2a754a027cd6
   