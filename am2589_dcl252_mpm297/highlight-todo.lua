function Str(el)
    if el.text:match("TODO:") then
      return pandoc.Span(pandoc.Str(el.text), {class = "todo"})
    elseif el.text:lower():match("picoscope") then
      return pandoc.Span(pandoc.Str(el.text), {class = "picoscope-highlight"})
    end
    return nil
  end
  